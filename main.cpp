// ============================================================
//  LuxPower Monitor - C++ / Dear ImGui Dashboard
//  Port từ script Node.js (Modbus over TCP, protocol 1/2)
//  Default IP: 192.168.1.20 : 8000  (protocol 1)
// ============================================================
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <GLFW/glfw3.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ============================================================
//  LuxPower protocol
// ============================================================
namespace lux {

static const uint8_t DEFAULT_DATALOG_SN[10] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint8_t EMPTY_INVERTER_SN[10] = {0};

static uint16_t crc16(const uint8_t* buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    }
    return crc;
}

static std::vector<uint8_t> buildReadInputFrame(uint16_t startAddr, uint16_t count, uint16_t protocol) {
    uint8_t cmd[18] = {0};
    cmd[0] = 0;                 // address
    cmd[1] = 4;                 // function R_INPUT
    memcpy(cmd + 2, EMPTY_INVERTER_SN, 10);
    cmd[12] = (uint8_t)(startAddr & 0xFF);
    cmd[13] = (uint8_t)(startAddr >> 8);
    cmd[14] = (uint8_t)(count & 0xFF);
    cmd[15] = (uint8_t)(count >> 8);
    uint16_t c = crc16(cmd, 16);
    cmd[16] = (uint8_t)(c & 0xFF);
    cmd[17] = (uint8_t)(c >> 8);

    std::vector<uint8_t> td(12 + 18);
    memcpy(td.data(), DEFAULT_DATALOG_SN, 10);
    td[10] = (uint8_t)(18 & 0xFF);
    td[11] = (uint8_t)(18 >> 8);
    memcpy(td.data() + 12, cmd, 18);

    uint16_t payloadLen = (uint16_t)(td.size() + 2);
    std::vector<uint8_t> frame(8 + td.size());
    frame[0] = 0xA1; frame[1] = 0x1A;
    frame[2] = (uint8_t)(protocol & 0xFF);
    frame[3] = (uint8_t)(protocol >> 8);
    frame[4] = (uint8_t)(payloadLen & 0xFF);
    frame[5] = (uint8_t)(payloadLen >> 8);
    frame[6] = 1;
    frame[7] = 194;             // TRANSLATE
    memcpy(frame.data() + 8, td.data(), td.size());
    return frame;
}

static int getRegister2(const std::vector<uint8_t>& frame, int index) {
    int p = index * 2 + 35;
    if (p + 1 >= (int)frame.size()) return 0;
    return (frame[p + 1] << 8) | frame[p];
}

} // namespace lux

// ============================================================
//  Shared state
// ============================================================
struct SharedState {
    std::mutex mtx;

    float pv = 0.f, consumption = 0.f, grid = 0.f, battery = 0.f;
    int   soc = 0;
    bool  connected = false;
    bool  online    = false;
    std::string deviceSn = "—";
    std::string lastError;
    double sessionStart = 0.0;

    std::deque<double> tHist;
    std::deque<float>  pvHist, loadHist, gridHist, battHist;
    static constexpr size_t HIST_MAX = 600;

    void pushHistory(double t, float pv_, float ld, float gr, float bt) {
        tHist.push_back(t);
        pvHist.push_back(pv_);
        loadHist.push_back(ld);
        gridHist.push_back(gr);
        battHist.push_back(bt);
        while (tHist.size() > HIST_MAX) {
            tHist.pop_front();
            pvHist.pop_front();
            loadHist.pop_front();
            gridHist.pop_front();
            battHist.pop_front();
        }
    }
};

// ============================================================
//  LuxClient – background thread
// ============================================================
class LuxClient {
public:
    explicit LuxClient(SharedState* s) : state_(s) {}
    ~LuxClient() { stop(); }

    void start(const std::string& host, int port, int protocol) {
        if (running_) return;
        host_ = host; port_ = port; protocol_ = protocol;
        running_ = true;
        thread_ = std::thread([this]{ run(); });
    }
    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

private:
    void run() {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);

        while (running_) {
            SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (s == INVALID_SOCKET) { sleepMs(1500); continue; }

            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_port   = htons((u_short)port_);
            inet_pton(AF_INET, host_.c_str(), &addr.sin_addr);

            if (connect(s, (sockaddr*)&addr, sizeof(addr)) != 0) {
                setStatus(false, "Không kết nối được tới " + host_);
                closesocket(s);
                sleepMs(1500);
                continue;
            }

            setStatus(true, "");

            std::vector<uint8_t> parserBuf;
            parserBuf.reserve(8192);

            auto lastSend = std::chrono::steady_clock::now() - std::chrono::seconds(10);
            auto sendRead = [&]() {
                auto frame = lux::buildReadInputFrame(0, 40, (uint16_t)protocol_);
                send(s, (const char*)frame.data(), (int)frame.size(), 0);
                lastSend = std::chrono::steady_clock::now();
            };
            sendRead();

            while (running_) {
                fd_set rfds; FD_ZERO(&rfds); FD_SET(s, &rfds);
                timeval tv{0, 150000}; // 150 ms
                int rv = select(0, &rfds, nullptr, nullptr, &tv);
                if (rv > 0 && FD_ISSET(s, &rfds)) {
                    uint8_t tmp[4096];
                    int n = recv(s, (char*)tmp, sizeof(tmp), 0);
                    if (n <= 0) break;
                    parserBuf.insert(parserBuf.end(), tmp, tmp + n);
                    parse(parserBuf);
                }
                if (std::chrono::steady_clock::now() - lastSend >= std::chrono::seconds(5))
                    sendRead();
            }

            closesocket(s);
            setStatus(false, "Mất kết nối");
            sleepMs(500);
        }
        WSACleanup();
    }

    void parse(std::vector<uint8_t>& buf) {
        while (true) {
            if (buf.size() < 2) return;
            if (buf[0] != 0xA1 || buf[1] != 0x1A) {
                size_t idx = 1;
                for (; idx < buf.size(); ++idx) if (buf[idx] == 0xA1) break;
                if (idx >= buf.size()) { buf.clear(); return; }
                buf.erase(buf.begin(), buf.begin() + idx);
                continue;
            }
            if (buf.size() < 6) return;
            uint16_t len = (uint16_t)(buf[4] | (buf[5] << 8));
            size_t fullLen = 6 + len;
            if (buf.size() < fullLen) return;

            std::vector<uint8_t> frame(buf.begin(), buf.begin() + fullLen);
            buf.erase(buf.begin(), buf.begin() + fullLen);

            if (frame.size() > 7 && frame[7] == 194) {
                int r7  = lux::getRegister2(frame, 7);
                int r8  = lux::getRegister2(frame, 8);
                int r9  = lux::getRegister2(frame, 9);
                int outInv  = lux::getRegister2(frame, 16);
                int inInv   = lux::getRegister2(frame, 17);
                int outGrid = lux::getRegister2(frame, 26);
                int inGrid  = lux::getRegister2(frame, 27);
                int soc     = lux::getRegister2(frame, 30);

                float pv = (float)(r7 + r8 + r9);
                float cons = (float)(outInv - inInv) + (float)(inGrid - outGrid);
                if (cons < 0) cons = 0;
                float grid = (float)(outGrid - inGrid);
                float batt = (float)(outInv - inInv);

                double now = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();

                std::lock_guard<std::mutex> lk(state_->mtx);
                state_->pv = pv;
                state_->consumption = cons;
                state_->grid = grid;
                state_->battery = batt;
                state_->soc = soc;
                state_->online = true;
                state_->lastError.clear();
                state_->pushHistory(now, pv, cons, grid, batt);
            }
        }
    }

    void setStatus(bool connected, const std::string& err) {
        std::lock_guard<std::mutex> lk(state_->mtx);
        state_->connected = connected;
        if (!connected) { state_->online = false; state_->lastError = err; }
    }

    static void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

    SharedState* state_;
    std::string  host_;
    int port_ = 8000;
    int protocol_ = 1;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

// ============================================================
//  Theme & helpers
// ============================================================
static void SetupTheme() {
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowRounding = 0.f;
    s.ChildRounding  = 14.f;
    s.FrameRounding  = 8.f;
    s.PopupRounding  = 10.f;
    s.ScrollbarRounding = 10.f;
    s.GrabRounding   = 8.f;
    s.WindowBorderSize = 0.f;
    s.ChildBorderSize  = 1.f;
    s.FramePadding   = ImVec2(12, 8);
    s.ItemSpacing    = ImVec2(10, 10);
    s.WindowPadding  = ImVec2(18, 18);

    auto* c = s.Colors;
    c[ImGuiCol_WindowBg]      = ImVec4(0.96f, 0.97f, 0.98f, 1.f);
    c[ImGuiCol_ChildBg]       = ImVec4(1.f, 1.f, 1.f, 1.f);
    c[ImGuiCol_Text]          = ImVec4(0.10f, 0.12f, 0.15f, 1.f);
    c[ImGuiCol_TextDisabled]  = ImVec4(0.55f, 0.58f, 0.62f, 1.f);
    c[ImGuiCol_Border]        = ImVec4(0.90f, 0.91f, 0.94f, 1.f);
    c[ImGuiCol_FrameBg]       = ImVec4(0.95f, 0.96f, 0.97f, 1.f);
    c[ImGuiCol_FrameBgHovered]= ImVec4(0.92f, 0.94f, 0.95f, 1.f);
    c[ImGuiCol_Button]        = ImVec4(0.13f, 0.60f, 0.35f, 1.f);
    c[ImGuiCol_ButtonHovered] = ImVec4(0.15f, 0.68f, 0.40f, 1.f);
    c[ImGuiCol_ButtonActive]  = ImVec4(0.11f, 0.52f, 0.30f, 1.f);
    c[ImGuiCol_Header]        = ImVec4(0.13f, 0.60f, 0.35f, 0.15f);
    c[ImGuiCol_ScrollbarBg]   = ImVec4(0.f, 0.f, 0.f, 0.f);
    c[ImGuiCol_ScrollbarGrab] = ImVec4(0.75f, 0.78f, 0.80f, 1.f);

    ImPlot::StyleColorsLight();
    ImPlotStyle& ps = ImPlot::GetStyle();
    ps.PlotPadding = ImVec2(10, 10);
    ps.LineWeight  = 2.2f;
    ps.PlotBorderSize = 0.f;
}

static void BeginCard(const char* id, ImVec2 size) {
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 14.f);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.f);
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(1.f, 1.f, 1.f, 1.f));
    ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.90f, 0.91f, 0.94f, 1.f));
    ImGui::BeginChild(id, size, true,
        ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
}
static void EndCard() {
    ImGui::EndChild();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(2);
}

static void StatCard(const char* id, const char* label, const char* value,
                     const char* sub, ImU32 accent, ImVec2 size) {
    BeginCard(id, size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 wp = ImGui::GetWindowPos();
    ImVec2 ws = ImGui::GetWindowSize();
    dl->AddRectFilled(wp, ImVec2(wp.x + 5.f, wp.y + ws.y), accent, 14.f, ImDrawFlags_RoundCornersLeft);

    ImGui::SetCursorPos(ImVec2(20, 14));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.45f, 0.48f, 0.53f, 1.f));
    ImGui::TextUnformatted(label);
    ImGui::PopStyleColor();

    ImGui::SetCursorPos(ImVec2(20, 38));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.08f, 0.10f, 0.13f, 1.f));
    ImGui::SetWindowFontScale(1.9f);
    ImGui::TextUnformatted(value);
    ImGui::SetWindowFontScale(1.f);
    ImGui::PopStyleColor();

    ImGui::SetCursorPos(ImVec2(20, 78));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.55f, 0.58f, 0.62f, 1.f));
    ImGui::TextUnformatted(sub);
    ImGui::PopStyleColor();

    EndCard();
}

static void ChartCard(const char* id, const char* title, ImU32 lineCol,
                      const std::deque<double>& t,
                      const std::deque<float>&  v,
                      const char* unit, ImVec2 size) {
    BeginCard(id, size);
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.10f, 0.12f, 0.15f, 1.f));
    ImGui::SetWindowFontScale(1.15f);
    ImGui::TextUnformatted(title);
    ImGui::SetWindowFontScale(1.f);
    ImGui::PopStyleColor();

    ImVec2 avail = ImGui::GetContentRegionAvail();
    if (ImPlot::BeginPlot("##plot", ImVec2(avail.x, avail.y - 4),
            ImPlotFlags_NoTitle | ImPlotFlags_NoLegend | ImPlotFlags_NoMenus)) {
        ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisFormat(ImAxis_Y1, "%.0f");
        ImPlot::PushStyleColor(ImPlotCol_Line, ImGui::ColorConvertU32ToFloat4(lineCol));
        ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.15f);

        if (!t.empty()) {
            // Ép CẢ HAI trục về double để tránh template ambiguous
            std::vector<double> xs(t.begin(), t.end());
            std::vector<double> ys(v.begin(), v.end());
            double t0 = xs.front();
            for (auto& x : xs) x -= t0;
            ImPlot::PlotShaded(unit, xs.data(), ys.data(), (int)xs.size(), 0.0);
            ImPlot::PlotLine(unit, xs.data(), ys.data(), (int)xs.size());
        }
        ImPlot::PopStyleVar();
        ImPlot::PopStyleColor();
        ImPlot::EndPlot();
    }
    EndCard();

}

static void DrawFlowPanel(const SharedState& s, ImVec2 size) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 wp = ImGui::GetWindowPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();

    float W = avail.x;
    float H = avail.y;
    ImVec2 center = ImVec2(wp.x + W * 0.5f, wp.y + H * 0.5f);

    auto node = [&](ImVec2 pos, ImVec2 sz, ImU32 col, const char* title, const char* val) {
        dl->AddRectFilled(ImVec2(pos.x, pos.y), ImVec2(pos.x + sz.x, pos.y + sz.y), col, 12.f);
        ImVec2 ts = ImGui::CalcTextSize(title);
        ImVec2 vs = ImGui::CalcTextSize(val);
        dl->AddText(ImVec2(pos.x + (sz.x - ts.x) * 0.5f, pos.y + 14), IM_COL32(255,255,255,220), title);
        dl->AddText(nullptr, 1.6f, ImVec2(pos.x + (sz.x - vs.x * 1.6f) * 0.5f, pos.y + 40),
                    IM_COL32(255,255,255,255), val);
    };

    char buf[64];
    ImU32 pvCol   = IM_COL32(0, 168, 112, 255);
    ImU32 gridCol = IM_COL32(52, 120, 246, 255);
    ImU32 loadCol = IM_COL32(255, 138, 0, 255);
    ImU32 battCol = IM_COL32(230, 60, 110, 255);

    ImVec2 pvPos   = ImVec2(wp.x + 30, wp.y + 20);
    ImVec2 loadPos = ImVec2(wp.x + W - 190, center.y - 45);
    ImVec2 gridPos = ImVec2(wp.x + 30, center.y - 45);
    ImVec2 battPos = ImVec2(wp.x + 30, wp.y + H - 95);

    snprintf(buf, sizeof(buf), "%.0f W", s.pv);
    node(pvPos, ImVec2(160, 90), pvCol, "PV", buf);
    snprintf(buf, sizeof(buf), "%.0f W", s.consumption);
    node(loadPos, ImVec2(160, 90), loadCol, "TẢI TIÊU THỤ", buf);
    snprintf(buf, sizeof(buf), "%.0f W", s.grid);
    node(gridPos, ImVec2(160, 90), gridCol, "LƯỚI ĐIỆN", buf);
    snprintf(buf, sizeof(buf), "%.0f W", s.battery);
    node(battPos, ImVec2(160, 90), battCol, "LƯU TRỮ", buf);

    auto arrow = [&](ImVec2 a, ImVec2 b, ImU32 col) {
        dl->AddLine(a, b, col, 3.0f);
        ImVec2 d(b.x - a.x, b.y - a.y);
        float L = sqrtf(d.x*d.x + d.y*d.y);
        if (L < 1) return;
        d.x /= L; d.y /= L;
        ImVec2 n(-d.y, d.x);
        ImVec2 p1(b.x - d.x*14 + n.x*7, b.y - d.y*14 + n.y*7);
        ImVec2 p2(b.x - d.x*14 - n.x*7, b.y - d.y*14 - n.y*7);
        dl->AddTriangleFilled(b, p1, p2, col);
    };

    ImVec2 fromPv(pvPos.x + 160, pvPos.y + 45);
    ImVec2 fromGrid(gridPos.x + 160, gridPos.y + 45);
    ImVec2 fromBatt(battPos.x + 160, battPos.y + 45);
    ImVec2 toLoad(loadPos.x, loadPos.y + 45);

    arrow(fromPv,   toLoad, pvCol);
    arrow(fromGrid, toLoad, gridCol);
    arrow(fromBatt, toLoad, battCol);

    dl->AddCircleFilled(center, 52, IM_COL32(255,255,255,255), 64);
    dl->AddCircle(center, 52, IM_COL32(220,225,230,255), 64, 2.f);
    char socTxt[16];
    snprintf(socTxt, sizeof(socTxt), "%d%%", s.soc);
    ImVec2 szSoc = ImGui::CalcTextSize(socTxt);
    dl->AddText(nullptr, 2.0f, ImVec2(center.x - szSoc.x, center.y - szSoc.y * 1.0f),
                IM_COL32(20,30,40,255), socTxt);
    const char* socLbl = "SOC";
    ImVec2 szLbl = ImGui::CalcTextSize(socLbl);
    dl->AddText(ImVec2(center.x - szLbl.x * 0.5f, center.y + 22), IM_COL32(120,130,140,255), socLbl);
}

// ============================================================
//  Main
// ============================================================
int main(int argc, char** argv) {
    // ================== CẤU HÌNH MẶC ĐỊNH ==================
    std::string host = "192.168.1.20";   //  <-- ĐÃ ĐỔI IP
    int port = 8000;
    int protocol = 1;

    // Cho phép override qua CLI:  LuxMonitor.exe <ip> <port> <protocol>
    if (argc > 1) host = argv[1];
    if (argc > 2) port = atoi(argv[2]);
    if (argc > 3) protocol = atoi(argv[3]);
    if (protocol != 1 && protocol != 2) protocol = 1;
    // =======================================================

    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    GLFWwindow* window = glfwCreateWindow(1360, 820, "LuxPower Monitor", nullptr, nullptr);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImFontConfig cfg;
    cfg.OversampleH = 2; cfg.OversampleV = 2;
    ImFont* fontMain = io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/segoeui.ttf", 17.0f, &cfg);
    if (!fontMain) io.Fonts->AddFontDefault();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    SetupTheme();

    SharedState state;
    state.sessionStart = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    LuxClient client(&state);
    client.start(host, port, protocol);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->Pos);
        ImGui::SetNextWindowSize(vp->Size);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(18, 18));
        ImGui::Begin("##root", nullptr,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollbar);
        ImGui::PopStyleVar();

        // ---------- HEADER ----------
        {
            ImGui::SetWindowFontScale(1.55f);
            ImGui::TextUnformatted("LuxPower Monitor");
            ImGui::SetWindowFontScale(1.f);
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.55f, 0.58f, 0.62f, 1.f));
            ImGui::Text("  •  %s:%d  (protocol %d)", host.c_str(), port, protocol);
            ImGui::PopStyleColor();

            bool online, connected;
            std::string err;
            {
                std::lock_guard<std::mutex> lk(state.mtx);
                online = state.online;
                connected = state.connected;
                err = state.lastError;
            }
            const char* statusTxt = online ? "Online" : (connected ? "Đang chờ dữ liệu..." : "Offline");
            ImU32 statusCol = online ? IM_COL32(0, 180, 100, 255)
                                     : (connected ? IM_COL32(240, 170, 20, 255)
                                                  : IM_COL32(220, 70, 70, 255));
            float rightW = ImGui::CalcTextSize(statusTxt).x + 34.f;
            ImGui::SameLine(ImGui::GetWindowWidth() - rightW - 20.f);
            ImDrawList* dl = ImGui::GetWindowDrawList();
            ImVec2 p = ImGui::GetCursorScreenPos();
            dl->AddCircleFilled(ImVec2(p.x + 8, p.y + 10), 6.f, statusCol);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 22);
            ImGui::TextUnformatted(statusTxt);

            if (!err.empty()) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.75f, 0.30f, 0.30f, 1.f));
                ImGui::Text("   %s", err.c_str());
                ImGui::PopStyleColor();
            }
        }
        ImGui::Separator();

        SharedState snap;
        {
            std::lock_guard<std::mutex> lk(state.mtx);
            snap.pv = state.pv; snap.consumption = state.consumption;
            snap.grid = state.grid; snap.battery = state.battery;
            snap.soc = state.soc;
            snap.online = state.online; snap.connected = state.connected;
            snap.deviceSn = state.deviceSn; snap.lastError = state.lastError;
            snap.tHist = state.tHist;
            snap.pvHist = state.pvHist;
            snap.loadHist = state.loadHist;
            snap.gridHist = state.gridHist;
            snap.battHist = state.battHist;
        }

        // ---------- STAT CARDS ----------
        {
            float totalW = ImGui::GetContentRegionAvail().x;
            float spacing = ImGui::GetStyle().ItemSpacing.x;
            float cardW = (totalW - spacing * 3.f) / 4.f;
            ImVec2 cardSize(cardW, 118);

            char v[32];
            snprintf(v, sizeof(v), "%.0f W", snap.pv);
            StatCard("##pv", "PV", v, "Sản lượng hôm nay  —", IM_COL32(0,168,112,255), cardSize);
            ImGui::SameLine();

            snprintf(v, sizeof(v), "%.0f W", snap.consumption);
            StatCard("##load", "TẢI TIÊU THỤ", v, "Điện tiêu thụ hôm nay  —", IM_COL32(255,138,0,255), cardSize);
            ImGui::SameLine();

            snprintf(v, sizeof(v), "%.0f W", snap.grid);
            StatCard("##grid", "LƯỚI ĐIỆN", v, "Lấy / Đẩy lưới  —", IM_COL32(52,120,246,255), cardSize);
            ImGui::SameLine();

            snprintf(v, sizeof(v), "%.0f W", snap.battery);
            StatCard("##batt", "LƯU TRỮ", v, "SOC  —", IM_COL32(230,60,110,255), cardSize);
        }

        // ---------- MAIN ROW ----------
        {
            float totalW = ImGui::GetContentRegionAvail().x;
            float totalH = ImGui::GetContentRegionAvail().y - 8.f;
            float spacing = ImGui::GetStyle().ItemSpacing.x;

            float leftW  = totalW * 0.55f;
            float rightW = totalW - leftW - spacing;

            BeginCard("##flow", ImVec2(leftW, totalH));
            DrawFlowPanel(snap, ImVec2(leftW, totalH));
            EndCard();

            ImGui::SameLine();

            ImGui::BeginGroup();
            {
                float chH = (totalH - spacing) * 0.5f;
                ChartCard("##pvchart",  "Công suất PV",  IM_COL32(0,168,112,255),
                          snap.tHist, snap.pvHist, "PV", ImVec2(rightW, chH));
                ChartCard("##ldchart",  "Tải tiêu thụ", IM_COL32(0,168,180,255),
                          snap.tHist, snap.loadHist, "Load", ImVec2(rightW, chH));
            }
            ImGui::EndGroup();
        }

        ImGui::End();

        ImGui::Render();
        int fbW, fbH;
        glfwGetFramebufferSize(window, &fbW, &fbH);
        glViewport(0, 0, fbW, fbH);
        glClearColor(0.96f, 0.97f, 0.98f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    client.stop();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
