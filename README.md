# reset
Bを押しながら挿すとリセット

# UDP
src以下にpasswd.hを追加し、下記の要領で設定
マイコンと同一ネットワークに居るホストPCを用意
HOSTはホストPCのIPとポート

```passwd.h
#define SSID "hugahuga"
#define PASSWD "hogehogehoge"
#define HOST_IP xxx,xxx,xxx,xxx // , 区切り
#define HOST_PORT xxxx
```

# 状態遷移
| 状態 | LED | 説明 |
| --- | --- | --- |
| STATE_INIT | 500 ms毎の点滅 | 起動初期状態 |
| STATE_ERROR | 100 ms毎の点滅 |  |
| STATE_AUTO | 点灯 |  |
| STATE_SEMIAUTO | 800 ms点灯 100 ms消灯の繰り返し |  |
| STATE_MANUAL | 消灯 | 初期状態からプロポ信号取得で遷移 |
| STATE_SBUS_LOST | 素早い2回点滅 | プロポ信号ロストで遷移（復帰あり） |
