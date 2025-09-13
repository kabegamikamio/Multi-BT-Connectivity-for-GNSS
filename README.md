# Multi BT Connectivity for GNSS

## 概要
Bluetoothクラシック(SPP)およびBLEを介してスマホと連携し、Track AddictやRace Chrono、GnssCircuitLoggerに位置情報を送信するためのArduinoスケッチです。

## 動作確認環境
- iOS (iPhone 15 Pro, iPhone SE 2)
  - GnssCircuitLogger (BLEモード)
- Android (Xperia XZ 601SO)
  - Track Addict (SPPモード)
  - Race Chrono (SPPモード)

## 使用方法
### iOSの場合
- GnssCircuitLoggerの設定から `KawaiiMyGNSSiOS` を選択します。
- 通常通りラップタイマーを起動してタイム計測を始めます。測位に成功している場合、マップ上で赤いピンが現在地に表示されます(スタート地点から遠い場合は `too far` と表示されると思います)。

### Android
*現時点ではSPPのみの対応です。将来的にはBLEでの対応を視野に入れています。*
Androidでは2通りのやり方がありますので、動作が安定する方を選んでください。

#### アプリと直接連携させる
- Bluetooth設定から ``KawaiiMyGNSSAndroid` とペアリングします。
- Track Addict や Race Chrono などのアプリから、Bluetooth GPS受信機に接続する設定をします。

#### 仮想位置情報を使う
*Androidでは本体GPSの代わりに外付けGPSなどを使って位置情報を上書きする機能があります。この機能を使うには「開発者向けオプション」を有効にする必要があります。*
- GPS Connector や Dragger GPS などの仮想位置情報に対応したアプリをインストールします。
- それらのGPSアプリから `KawaiiMyGNSSAndroid` に接続します。
- GPSアプリからOSに仮想位置情報を提供するよう設定します。
- スマホ本体設定で、`開発者向けオプション -> 仮の現在地情報アプリを選択` からGPSアプリを選択します。
- Track Addict や Race Chrono では内蔵GPSを使う設定を選びます。

## プログラム使用に伴う規約
- 改変しない場合に限り再頒布を認めます。ただし、作者情報を明記してください。
- 個人利用かつ再頒布しない限り改変を認めます。ただし、作者の許可がある場合には、改変したプログラムを再頒布することを認める場合があります。
- 商用利用はお断りします。
- プログラム利用に関する損害について、作者は一切の責任を負いません。
- その他、利用や改変、再頒布についてわからないことがありましたら、Issuesからお問い合わせください。
