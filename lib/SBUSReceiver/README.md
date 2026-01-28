# Only for use ESP
ここがESPに合わせてあるため、ESP特化のプログラムです。
末尾trueによりuart信号の反転が可能です。

```c++
void SBUSReceiver::begin() {
    serial_.begin(SBUS_BAUD, SERIAL_8E2, rxPin_, txPin_, true);
}
```
# Program bloking
受信が完了するまでブロッキングするプログラムになっています。
通常 14 ms, ハイスピードで 7 ms です。

```c++
bool SBUSReceiver::readFrame() {
    static uint8_t idx = 0;

    while (serial_.available()) {
        uint8_t c = serial_.read();

        if (idx == 0 && c != 0x0F) continue;

        sbusData_[idx++] = c;

        if (idx >= 25) {
            idx = 0;
            decode(sbusData_);
            return true;
        }
    }
    return false;
}
```

sbus.begin()
Serial.beginをする

# CH

1 0 右　横
2 1 左　縦
3 2 右　縦
4 3 左　横
5 4 SG 352-1024-1696 下-中-上
6 5 = C1
7 6 = inv C3
8 7 SE 352-1024-1696 下-中-上
9 8
10 9
11 10
12 11
13 12
14 13
15 14
16 15 プロポ電源 ON:1792 OFF:195
17 SE

トリムで4ずつ動く

1696 -> +60
352 -> -60

x = deg
y = sbus data
y = 11.2*x + 1024

x = (y-1024)/11.2