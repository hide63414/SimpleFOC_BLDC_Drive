# SimpleFOC_BLDC_Drive

## AtomS3_SimpleFOC_ESPNOW_Drive
AtomS3でクローズドループで速度制御<BR>
・速度指定<BR>
　シリアル通信でTコマンド送信<BR>
　ESPNOWで接続して指定<BR>

## Dial_SimpleFOC_ESPNOW_Command
M5DialからESPNOWで速度指令、受信側はAtomS3_SimpleFOC_ESPNOW_Drive<BR>
・M5Dialから速度指令<BR>
・AtomS3は指定された速度、現在速度、現在電流、IMUデータをM5Dialに送信

## AtomS3_SimpleFOCStudio
SimpleFOC Studioでモーター制御<BR>
速度モードや位置モードの切り替え、ターゲット値の指定、PIDパラメータ変更<BR>
動作状況を見ながら調整できるので便利<BR>
調整後の値をArudinoプログラム形式で出力できるので、コードに貼り付ければいい<BR>
