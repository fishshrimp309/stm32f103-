#include <stdio.h>
#include <stdint.h>
#include <windows.h>
#include <conio.h>

// MinGW 可能未定义此常量
#ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#endif

// =================== 可调整宏定义参数 ===================
#define MAX_PORTS       32          // 最大扫描端口数
#define SEND_INTERVAL   2000          // 定时发送间隔 (单位: ms)

#define VAL_MIN         0           // 油门值最小值
#define VAL_MAX         255         // 油门值最大值 (对应STM32端 0~255→1000~2000)
#define VAL_STEP        5           // 滚轮每格改变的步长
#define VAL_INIT        0         // 油门的初始值 (中间值→约1500)
// ============================================================
// ---------- 按键位映射表 (12个键, 对应 KeyType 枚举) ----------
// 每个键对应bitmap中的一个bit位, 按下时该位=1
// 低位在前: bit0=W, bit1=S, bit2=A, bit3=D, bit4=Shift, bit5=Ctrl,
//           bit6=Q, bit7=E, bit8=R, bit9=F, bit10=Z, bit11=X
const int TRACKED_KEYS[] = {
    'W',            // bit 0  - Key_W
    'S',            // bit 1  - Key_S
    'A',            // bit 2  - Key_A
    'D',            // bit 3  - Key_D
    VK_SHIFT,       // bit 4  - Key_Shift
    VK_CONTROL,     // bit 5  - Key_Ctrl
    'Q',            // bit 6  - Key_Q
    'E',            // bit 7  - Key_E
    'R',            // bit 8  - Key_R
    'F',            // bit 9  - Key_F
    'Z',            // bit 10 - Key_Z
    'X',            // bit 11 - Key_X
};
const int NUM_TRACKED_KEYS = sizeof(TRACKED_KEYS) / sizeof(TRACKED_KEYS[0]);

// 返回键名 (用于屏幕显示)
const char* getKeyName(int vk) {
    if (vk >= 'A' && vk <= 'Z') {
        static char name[2] = {0};
        name[0] = (char)vk; return name;
    }
    if (vk >= '0' && vk <= '9') {
        static char name[2] = {0};
        name[0] = (char)vk; return name;
    }
    switch (vk) {
        case VK_SHIFT:  return "SHF";
        case VK_CONTROL:return "CTL";
        default:        return "??";
    }
}

// 扫描可用端口
int scanSerialPorts(int foundPorts[]) {
    int count = 0;
    char portName[20];
    for (int i = 1; i <= MAX_PORTS; i++) {
        sprintf(portName, "\\\\.\\COM%d", i);
        HANDLE hPort = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
        if (hPort != INVALID_HANDLE_VALUE) {
            foundPorts[count++] = i;
            CloseHandle(hPort);
        } else if (GetLastError() == ERROR_ACCESS_DENIED) {
            foundPorts[count++] = i;
        }
    }
    return count;
}

// 检查串口状态
int isPortAlive(HANDLE hSerial) {
    DWORD errors;
    COMSTAT comStat;
    return ClearCommError(hSerial, &errors, &comStat);
}

int main() {
    // 设置控制台编码为 UTF-8, 解决中文乱码
    system("chcp 65001 > nul");
    SetConsoleOutputCP(CP_UTF8);

    // 启用 ANSI/VT100 转义序列 (支持 \033[1A 上移光标等)
    HANDLE hOutput = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD outMode;
    GetConsoleMode(hOutput, &outMode);
    SetConsoleMode(hOutput, outMode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);

    // 获取标准输入句柄
    HANDLE hInput = GetStdHandle(STD_INPUT_HANDLE);
    DWORD mode;
    GetConsoleMode(hInput, &mode);

    // 启用鼠标输入 (ENABLE_MOUSE_INPUT)
    // 关闭快速编辑模式, 防止鼠标点击暂停程序
    SetConsoleMode(hInput, (mode & ~ENABLE_QUICK_EDIT_MODE) | ENABLE_MOUSE_INPUT);

    while (1) {
        int foundPorts[MAX_PORTS];
        int portCount = scanSerialPorts(foundPorts);

        if (portCount == 0) {
            printf("\n[提示] 未找到任何可用串口, 5秒后自动重新扫描...\n");
            for (int i = 0; i < 50; i++) {
                if (_kbhit() && _getch() == 27) return 0;
                Sleep(100);
            }
            system("cls");
            continue;
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
        if (hSerial == INVALID_HANDLE_VALUE) {
            printf("\n[错误] 无法打开 COM%d\n", selectedComNum);
            Sleep(2000); system("cls"); continue;
        }

        // 配置 115200
        DCB dcb = {0};
        dcb.DCBlength = sizeof(dcb);
        GetCommState(hSerial, &dcb);
        dcb.BaudRate = CBR_115200; dcb.ByteSize = 8; dcb.StopBits = ONESTOPBIT; dcb.Parity = NOPARITY;
        SetCommState(hSerial, &dcb);

        // 设置串口读取超时 (短等待: 最多等 10ms, 字节间隔 2ms)
        COMMTIMEOUTS timeouts;
        timeouts.ReadIntervalTimeout = 2;          // 字节间隔超时 2ms
        timeouts.ReadTotalTimeoutMultiplier = 0;
        timeouts.ReadTotalTimeoutConstant = 10;    // 总超时 10ms
        SetCommTimeouts(hSerial, &timeouts);

        printf("\n============================================\n");
        printf("成功连接到 COM%d, 波特率: 115200\n", selectedComNum);
        printf("------------------------------------------------\n");
        printf("[发送] 每 %d ms / 按键滚轮变化时 → 8字节(0xAA)\n", SEND_INTERVAL);
        printf("  字节0:0xAA | 字节1-3:按键位图 | 字节4:油门 | 5-7:保留\n");
        printf("[接收] 实时解析 → 8字节(0xBB) 回传姿态数据\n");
        printf("  字节0:0xBB | 1:Roll | 2:Pitch | 3:Yaw | 4-5:Thr(16bit) | 6-7:保留\n");
        printf("------------------------------------------------\n");
        printf("[操作] 滚轮=油门 | 按键=长按检测 | Esc=断开\n");
        printf("============================================\n\n");

        // 清空缓冲区
        while (_kbhit()) _getch();
        FlushConsoleInputBuffer(hInput);

        int connectionLost = 0;
        int current_value = VAL_INIT;
        unsigned int lastKeyBitmap = 0;  // 记录上一次按键位图, 用于检测变化

        // 回传数据 (从 STM32 接收)
        int rx_roll = 0, rx_pitch = 0, rx_yaw = 0, rx_throttle = 0;
        int rx_updated = 0;

        unsigned char rxAccum[256];
        DWORD rxAccumLen = 0;

        DWORD lastSendTime = GetTickCount();

        printf("\r[回传] Roll:%4d Pitch:%4d Yaw:%4d Thr:%4d    \n\r[发送] 油门:%3d | 按键:[无]    ",
               rx_roll, rx_pitch, rx_yaw, rx_throttle, current_value);
        fflush(stdout);

        while (1) {
            // 1. 硬件连接检测
            if (!isPortAlive(hSerial)) { connectionLost = 1; break; }

            // 2. 键盘检测 → 仅检测 Esc 退出 (按键状态通过定时8字节包上报)
            if (_kbhit()) {
                if (_getch() == 27) { // Esc
                    printf("\n用户主动断开当前连接。\n");
                    break;
                }
            }

            // 3. 鼠标滚轮事件 → 实时调整油门值
            int valueChanged = 0;
            DWORD numEvents = 0;
            GetNumberOfConsoleInputEvents(hInput, &numEvents);

            if (numEvents > 0) {
                INPUT_RECORD inputBuffer[32];
                DWORD numEventsRead = 0;
                if (ReadConsoleInput(hInput, inputBuffer, 32, &numEventsRead)) {
                    for (DWORD i = 0; i < numEventsRead; i++) {
                        if (inputBuffer[i].EventType == MOUSE_EVENT &&
                            inputBuffer[i].Event.MouseEvent.dwEventFlags == MOUSE_WHEELED) {

                            short wheelDelta = (short)HIWORD(inputBuffer[i].Event.MouseEvent.dwButtonState);

                            if (wheelDelta > 0) {
                                if (current_value + VAL_STEP <= VAL_MAX) {
                                    current_value += VAL_STEP;
                                    valueChanged = 1;
                                }
                            } else if (wheelDelta < 0) {
                                if (current_value - VAL_STEP >= VAL_MIN) {
                                    current_value -= VAL_STEP;
                                    valueChanged = 1;
                                }
                            }
                        }
                    }
                }
            }

            // 4. 构建并发送 8 字节数据包 (按键变化/滚轮变化实时发, 或定时发)
            DWORD currentTime = GetTickCount();

            // 扫描所有跟踪按键, 构建 24 位位图
            unsigned int keyBitmap = 0;
            for (int i = 0; i < NUM_TRACKED_KEYS; i++) {
                if (GetAsyncKeyState(TRACKED_KEYS[i]) & 0x8000) {
                    keyBitmap |= (1 << i);
                }
            }
            int keyChanged = (keyBitmap != lastKeyBitmap);

            int needSend = (valueChanged) || keyChanged || (currentTime - lastSendTime >= SEND_INTERVAL);

            if (needSend) {

                // ---- 构建 8 字节数据包 ----
                unsigned char packet[8] = {0};
                packet[0] = 0xAA;  // 帧头

                // STM32端解析: current_key = Buf[1]<<16 | Buf[2]<<8 | Buf[3]
                // 所以 packet[1]=高字节(bits16-23), packet[3]=低字节(bits0-7)
                packet[1] = (unsigned char)((keyBitmap >> 16) & 0xFF);
                packet[2] = (unsigned char)((keyBitmap >> 8)  & 0xFF);
                packet[3] = (unsigned char)( keyBitmap        & 0xFF);

                // 字节4: 油门值
                packet[4] = (unsigned char)current_value;

                // 字节5-7: 保留 (已初始化为0)

                // ---- 发送 ----
                DWORD bytesWritten;
                if (!WriteFile(hSerial, packet, 8, &bytesWritten, NULL)) {
                    connectionLost = 1;
                    break;
                }
                // ---- 屏幕显示 (仅更新发送行) ----
                printf("\r[发送] 油门:%3d | 按键:[", current_value);
                int pressedCount = 0;
                for (int i = 0; i < NUM_TRACKED_KEYS; i++) {
                    if (keyBitmap & (1 << i)) {
                        if (pressedCount > 0) printf("+");
                        printf("%s", getKeyName(TRACKED_KEYS[i]));
                        pressedCount++;
                    }
                }
                if (pressedCount == 0) printf("无");
                printf("]    ");
                fflush(stdout);

                lastSendTime = currentTime;
                lastKeyBitmap = keyBitmap;
            }

            // 5. 读取串口回传数据 → 累积缓冲区, 拼完整包再解析
            {
                unsigned char tmp[64];
                DWORD bytesRead = 0;
                if (ReadFile(hSerial, tmp, sizeof(tmp), &bytesRead, NULL) && bytesRead > 0) {
                    // 追加到累积缓冲区
                    if (rxAccumLen + bytesRead <= sizeof(rxAccum)) {
                        memcpy(rxAccum + rxAccumLen, tmp, bytesRead);
                        rxAccumLen += bytesRead;
                    } else {
                        // 缓冲区溢出, 清空重新开始
                        rxAccumLen = 0;
                    }
                }

                // 在累积缓冲中滑动搜索 0xBB 帧头
                DWORD pos = 0;
                while (pos + 7 < rxAccumLen) {
                    if (rxAccum[pos] == 0xBB) {
                        rx_roll    = (signed char)rxAccum[pos + 1];
                        rx_pitch   = (signed char)rxAccum[pos + 2];
                        rx_yaw     = (signed char)rxAccum[pos + 3];
                        rx_throttle = (int16_t)(rxAccum[pos + 4] | ((int16_t)rxAccum[pos + 5] << 8));
                        rx_updated = 1;
                        pos += 8;  // 跳过本包, 继续搜索下一包
                    } else {
                        pos++;
                    }
                }

                // 将未解析的尾部数据移到缓冲区开头
                if (pos > 0 && pos < rxAccumLen) {
                    memmove(rxAccum, rxAccum + pos, rxAccumLen - pos);
                    rxAccumLen -= pos;
                } else if (pos >= rxAccumLen) {
                    rxAccumLen = 0;
                }
            }

            // 6. 屏幕刷新 (收到回传数据时, 上移一行更新两行)
            if (rx_updated) {
                rx_updated = 0;
                printf("\033[1A\r[回传] Roll:%4d Pitch:%4d Yaw:%4d Thr:%4d    \n\r[发送] 油门:%3d | 按键:[",
                       rx_roll, rx_pitch, rx_yaw, rx_throttle, current_value);
                int pc = 0;
                for (int i = 0; i < NUM_TRACKED_KEYS; i++) {
                    if (GetAsyncKeyState(TRACKED_KEYS[i]) & 0x8000) {
                        if (pc > 0) printf("+");
                        printf("%s", getKeyName(TRACKED_KEYS[i]));
                        pc++;
                    }
                }
                if (pc == 0) printf("无");
                printf("]    ");
                fflush(stdout);
            }

            Sleep(5); // 降低CPU占用
        }

        CloseHandle(hSerial);
        if (connectionLost) {
            printf("\n[错误] COM%d 已断开, 3秒后重新扫描...\n", selectedComNum);
            Sleep(3000);
        }
        system("cls");
    }
    return 0;
}
