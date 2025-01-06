# SimpleFOC_BLDC_Drive

## AtomS3_SimpleFOC_ESPNOW_Drive
AtomS3でクローズドループで速度制御<BR>
・速度指定<BR>
　シリアル通信でTコマンド送信<BR>
　ESPNOWで接続して指定<BR>

・SimpleFOC Stuioから操作可能<BR>
　motor.moveの()内を指定しないように変更
```
　  motor.move(targetVelocity);
  　//motor.move();<BR>
```

## Dial_SimpleFOC_ESPNOW_Command
M5DialからESPNOWで速度指令、受信側はAtomS3_SimpleFOC_ESPNOW_Drive<BR>
・M5Dialから速度指令
・AtomS3は指定された速度、現在速度、現在電流、IMUデータをM5Dialに送信

## AtomS3_SimpleFOCStudio
SimpleFOC Studioでモーター制御
速度モードや位置モードの切り替え、ターゲット値の指定、PIDパラメータ変更
動作状況を見ながら調整できるので便利
調整後の値をArudinoプログラム形式で出力できるので、コードに貼り付ければいい

モータ制御はAtomS3_SimpleFOC_ESPNOW_Driveを使用
Loop内のmoveコマンドの()だけにしておく
```
motor.moveの()
```