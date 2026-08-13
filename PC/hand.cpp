#include <stdio.h>
#include <stdint.h>
#include <windows.h>
#include <conio.h>
#include <xinput.h>

// MinGW 可能未定义此常量
#ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#endif

// =================== 可调整宏定义参数 ===================
#define MAX_PORTS       32          // 最大扫描端口数
#define SEND_INTERVAL   50          // 定时发送间隔 (单位: ms)

#define VAL_MIN         0           // 摇杆最小值 (-1.0)
#define VAL_MAX         255         // 摇杆最大值 (1.0)
#define VAL_STEP        5           // 滚轮每格改变的步长
#define VAL_INIT        127         // 初始中心值 (127 对应 0.0)
// ============================================================

// ---------- 动态加载 XInput API (防止编译时找不到库) ----------
typedef DWORD(WINAPI* XInputGetState_t)(DWORD, XINPUT_STATE*);
XInputGetState_t pXInputGetState = NULL;

void initXInput() {
    HMODULE hModule = LoadLibraryA("xinput1_4.dll");       // Win8+
    if (!hModule) hModule = LoadLibraryA("xinput1_3.dll"); // Win7
    if (!hModule) hModule = LoadLibraryA("xinput9_1_0.dll");// Vista
    if (hModule) {
        pXInputGetState = (XInputGetState_t)GetProcAddress(hModule, "XInputGetState");
    }
}

// ---------- 摇杆映射函数 (-32768~32767 映射到 0~255) ----------
unsigned char map_joystick_to_byte(short raw_axis) {
    const short deadzone = 2000; // 摇杆死区

    if (raw_axis > deadzone) {
        // 正半轴映射：2000~32767 -> 127~255
        return 127 + (raw_axis - deadzone) * 128 / (32767 - deadzone);
    }
    else if (raw_axis < -deadzone) {
        // 负半轴映射：-32768~-2000 -> 0~127
        return 127 - (-raw_axis - deadzone) * 127 / (32768 - deadzone);
    }
    // 死区内回中
    return 127;
}

// ---------- 按键位映射表 (12个键) ----------
const int TRACKED_KEYS[] = {
    'W', 'S', 'A', 'D', VK_SHIFT, VK_CONTROL,
    'Q', 'E', 'R', 'F', 'I', 'P'
};
const int NUM_TRACKED_KEYS = sizeof(TRACKED_KEYS) / sizeof(TRACKED_KEYS[0]);

const char* getKeyName(int vk) {
    if (vk >= 'A' && vk <= 'Z') { static char name[2] = { 0 }; name[0] = (char)vk; return name; }
    if (vk >= '0' && vk <= '9') { static char name[2] = { 0 }; name[0] = (char)vk; return name; }
    switch (vk) {
    case VK_SHIFT:  return "SHF";
    case VK_CONTROL:return "CTL";
    default:        return "??";
    }
}

int scanSerialPorts(int foundPorts[]) {
    int count = 0;
    char portName[20];
    for (int i = 1; i <= MAX_PORTS; i++) {
        sprintf(portName, "\\\\.\\COM%d", i);
        HANDLE hPort = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
        if (hPort != INVALID_HANDLE_VALUE) { foundPorts[count++] = i; CloseHandle(hPort); }
        else if (GetLastError() == ERROR_ACCESS_DENIED) { foundPorts[count++] = i; }
    }
    return count;
}

int isPortAlive(HANDLE hSerial) {
    DWORD errors; COMSTAT comStat;
    return ClearCommError(hSerial, &errors, &comStat);
}

int main() {
    // system("chcp 65001 > nul");
    // SetConsoleOutputCP(CP_UTF8);

    HANDLE hOutput = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD outMode; GetConsoleMode(hOutput, &outMode);
    SetConsoleMode(hOutput, outMode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);

    HANDLE hInput = GetStdHandle(STD_INPUT_HANDLE);
    DWORD mode; GetConsoleMode(hInput, &mode);
    SetConsoleMode(hInput, (mode & ~ENABLE_QUICK_EDIT_MODE) | ENABLE_MOUSE_INPUT);

    initXInput(); // 初始化手柄支持

    while (1) {
        int foundPorts[MAX_PORTS];
        int portCount = scanSerialPorts(foundPorts);

        if (portCount == 0) {
            printf("\n[提示] 未找到可用串口, 5秒后重试...\n");
            Sleep(5000); system("cls"); continue;
        }

        printf("\n请选择可用端口:\n");
        for (int i = 0; i < portCount; i++) printf("[%d] COM%d\n", i + 1, foundPorts[i]);
        printf("[%d] 重新扫描端口\n(按 Esc 退出程序)\n", portCount + 1);

        int choice = 0, shouldQuit = 0;
        while (choice < 1 || choice > portCount + 1) {
            printf("\n请选择要连接的端口编号 (1-%d): ", portCount + 1);
            if (_kbhit() && _getch() == 27) { shouldQuit = 1; break; }
            if (scanf("%d", &choice) != 1) while (getchar() != '\n');
        }

        if (shouldQuit) break;
        if (choice == portCount + 1) { system("cls"); continue; }

        int selectedComNum = foundPorts[choice - 1];
        char chosenPortName[20];
        sprintf(chosenPortName, "\\\\.\\COM%d", selectedComNum);

        HANDLE hSerial = CreateFile(chosenPortName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (hSerial == INVALID_HANDLE_VALUE) { printf("\n[错误] 无法打开 COM%d\n", selectedComNum); Sleep(2000); system("cls"); continue; }

        DCB dcb = { 0 }; dcb.DCBlength = sizeof(dcb); GetCommState(hSerial, &dcb);
        dcb.BaudRate = CBR_115200; dcb.ByteSize = 8; dcb.StopBits = ONESTOPBIT; dcb.Parity = NOPARITY;
        SetCommState(hSerial, &dcb);

        COMMTIMEOUTS timeouts;
        timeouts.ReadIntervalTimeout = 2; timeouts.ReadTotalTimeoutMultiplier = 0; timeouts.ReadTotalTimeoutConstant = 10;
        SetCommTimeouts(hSerial, &timeouts);

        printf("\n============================================\n");
        printf("成功连接到 COM%d, 波特率: 115200\n", selectedComNum);
        printf("[支持设备] 键盘 / 鼠标滚轮 / 盖世小鸡(Xbox)手柄\n");
        printf("  字节4:左摇杆Y(油门) | 字节5:右摇杆Y(前后)\n");
        printf("  字节6:右摇杆X(左右) | 字节7:左摇杆X(转向)\n");
        printf("  * 手柄 A键=P, B键=I | 中心值=127 (对应 0.0)\n");
        printf("============================================\n\n");

        while (_kbhit()) _getch();
        FlushConsoleInputBuffer(hInput);

        int connectionLost = 0;
        
        // 四个通道的初始值为 127 (即 0.0)
        int current_throttle = VAL_INIT; // 字节4: 左Y
        int right_stick_y    = VAL_INIT; // 字节5: 右Y
        int right_stick_x    = VAL_INIT; // 字节6: 右X
        int left_stick_x     = VAL_INIT; // 字节7: 左X
        
        int was_gamepad_active = 0;
        unsigned int lastKeyBitmap = 0;
        int rx_roll = 0, rx_pitch = 0, rx_yaw = 0, rx_throttle = 0, rx_updated = 0;
        unsigned char rxAccum[256]; DWORD rxAccumLen = 0;
        DWORD lastSendTime = GetTickCount();

        // 初始界面打印
        printf("\r[回传] Roll:%4d Pitch:%4d Yaw:%4d Thr:%4d    \n\r[发送] 左Y:%3d 右Y:%3d 右X:%3d 左X:%3d | 按键:[无]    ",
            rx_roll, rx_pitch, rx_yaw, rx_throttle, 
            current_throttle, right_stick_y, right_stick_x, left_stick_x);
        fflush(stdout);

        while (1) {
            if (!isPortAlive(hSerial)) { connectionLost = 1; break; }
            if (_kbhit() && _getch() == 27) break;

            int valueChanged = 0;
            
            // ================= 1. 键盘状态 =================
            unsigned int keyBitmap = 0;
            for (int i = 0; i < NUM_TRACKED_KEYS; i++) {
                if (GetAsyncKeyState(TRACKED_KEYS[i]) & 0x8000) keyBitmap |= (1 << i);
            }

            // ================= 2. 手柄状态 =================
            if (pXInputGetState) {
                XINPUT_STATE state;
                if (pXInputGetState(0, &state) == ERROR_SUCCESS) {
                    
                    if (state.Gamepad.wButtons & XINPUT_GAMEPAD_B) { break; }
                    if (state.Gamepad.wButtons & XINPUT_GAMEPAD_A) keyBitmap |= (1 << 11);
                    if (state.Gamepad.wButtons & XINPUT_GAMEPAD_X) keyBitmap |= (1 << 11);
                    if (state.Gamepad.wButtons & XINPUT_GAMEPAD_Y) keyBitmap |= (1 << 10);
                    // 映射所有 4 个轴
                    int ly = map_joystick_to_byte(state.Gamepad.sThumbLY);
                    int ry = map_joystick_to_byte(state.Gamepad.sThumbRY);
                    int rx = map_joystick_to_byte(state.Gamepad.sThumbRX);
                    int lx = map_joystick_to_byte(state.Gamepad.sThumbLX);

                    if (ly != 127 || ry != 127 || rx != 127 || lx != 127) {
                        current_throttle = ly;
                        right_stick_y = ry;
                        right_stick_x = rx;
                        left_stick_x = lx;
                        was_gamepad_active = 1;
                        valueChanged = 1;
                    } else if (was_gamepad_active) {
                        // 摇杆回到中心死区，自动回中
                        current_throttle = 127;
                        right_stick_y = 127;
                        right_stick_x = 127;
                        left_stick_x = 127;
                        was_gamepad_active = 0;
                        valueChanged = 1;
                    }
                }
            }

            // ================= 3. 鼠标滚轮 (仅控制左Y油门) =================
            DWORD numEvents = 0; GetNumberOfConsoleInputEvents(hInput, &numEvents);
            if (numEvents > 0) {
                INPUT_RECORD inputBuffer[32]; DWORD numEventsRead = 0;
                if (ReadConsoleInput(hInput, inputBuffer, 32, &numEventsRead)) {
                    for (DWORD i = 0; i < numEventsRead; i++) {
                        if (inputBuffer[i].EventType == MOUSE_EVENT && inputBuffer[i].Event.MouseEvent.dwEventFlags == MOUSE_WHEELED) {
                            short wheelDelta = (short)HIWORD(inputBuffer[i].Event.MouseEvent.dwButtonState);
                            if (wheelDelta > 0 && current_throttle + VAL_STEP <= VAL_MAX) { current_throttle += VAL_STEP; valueChanged = 1; }
                            else if (wheelDelta < 0 && current_throttle - VAL_STEP >= VAL_MIN) { current_throttle -= VAL_STEP; valueChanged = 1; }
                        }
                    }
                }
            }

            // ================= 4. 特殊按键逻辑 (急停) =================
            // 如果按下了 P 键 (键盘P 或 手柄A)，强制所有轴回到中心值 127 (即 STM32端的 0.0)
            if (keyBitmap & (1 << 11)) {
                if (current_throttle != 127 || right_stick_y != 127 || right_stick_x != 127 || left_stick_x != 127) { 
                    current_throttle = 127; 
                    right_stick_y = 127;
                    right_stick_x = 127;
                    left_stick_x = 127;
                    valueChanged = 1; 
                }
            }

            // ================= 5. 发送数据 =================
            DWORD currentTime = GetTickCount();
            int keyChanged = (keyBitmap != lastKeyBitmap);

            // 判断是否需要发送：
            // 1. 数据变了，并且距离上次发送已经超过了 15ms (防止动摇杆时疯狂发包堵死STM32)
            // 2. 或者就算数据没变，也到了定时心跳时间 (SEND_INTERVAL，比如 50ms)
            int dataChanged = valueChanged || keyChanged;
            if ( (dataChanged && (currentTime - lastSendTime >= 10)) || 
                (currentTime - lastSendTime >= SEND_INTERVAL) ) {
                
                unsigned char packet[8] = { 0 };
                packet[0] = 0xAA; // 帧头
                packet[1] = (unsigned char)((keyBitmap >> 16) & 0xFF);
                packet[2] = (unsigned char)((keyBitmap >> 8) & 0xFF);
                packet[3] = (unsigned char)(keyBitmap & 0xFF);
                packet[4] = (unsigned char)current_throttle; // 字节4: 左摇杆Y
                packet[5] = (unsigned char)right_stick_y;    // 字节5: 右摇杆Y
                packet[6] = (unsigned char)right_stick_x;    // 字节6: 右摇杆X
                packet[7] = (unsigned char)left_stick_x;     // 字节7: 左摇杆X

                DWORD bytesWritten;
                if (!WriteFile(hSerial, packet, 8, &bytesWritten, NULL)) { connectionLost = 1; break; }

                // 打印发送行信息
                printf("\r[发送] 左Y:%3d 右Y:%3d 右X:%3d 左X:%3d | 按键:[", 
                    current_throttle, right_stick_y, right_stick_x, left_stick_x);
                
                int pressedCount = 0;
                for (int i = 0; i < NUM_TRACKED_KEYS; i++) {
                    if (keyBitmap & (1 << i)) {
                        if (pressedCount > 0) printf("+");
                        printf("%s", getKeyName(TRACKED_KEYS[i]));
                        pressedCount++;
                    }
                }
                if (pressedCount == 0) printf("无");
                printf("]          "); 
                fflush(stdout);

                lastSendTime = currentTime; 
                lastKeyBitmap = keyBitmap;
            }

            // ================= 6. 接收回传 (原样保留) =================
            unsigned char tmp[64]; DWORD bytesRead = 0;
            if (ReadFile(hSerial, tmp, sizeof(tmp), &bytesRead, NULL) && bytesRead > 0) {
                if (rxAccumLen + bytesRead <= sizeof(rxAccum)) { memcpy(rxAccum + rxAccumLen, tmp, bytesRead); rxAccumLen += bytesRead; }
                else rxAccumLen = 0;
            }
            DWORD pos = 0;
            while (pos + 7 < rxAccumLen) {
                if (rxAccum[pos] == 0xBB) {
                    rx_roll = (signed char)rxAccum[pos + 1]; rx_pitch = (signed char)rxAccum[pos + 2]; rx_yaw = (signed char)rxAccum[pos + 3];
                    rx_throttle = (int16_t)(rxAccum[pos + 4] | ((int16_t)rxAccum[pos + 5] << 8));
                    rx_updated = 1; pos += 8;
                } else pos++;
            }
            if (pos > 0 && pos < rxAccumLen) { memmove(rxAccum, rxAccum + pos, rxAccumLen - pos); rxAccumLen -= pos; }
            else if (pos >= rxAccumLen) rxAccumLen = 0;

            if (rx_updated) {
                rx_updated = 0;
                printf("\033[1A\r[回传] Roll:%4d Pitch:%4d Yaw:%4d Thr:%4d    \n", rx_roll, rx_pitch, rx_yaw, rx_throttle);
                fflush(stdout);
            }
            Sleep(5);
        }
        CloseHandle(hSerial);
        if (connectionLost) { printf("\n[错误] COM 已断开...\n"); Sleep(3000); }
        system("cls");
    }
    return 0;
}
