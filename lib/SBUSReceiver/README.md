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
1 右　横
2 左　縦
3 右　縦
4 左　横
5 SG
6 左　横
9 左裏

13 SD
17 SE
