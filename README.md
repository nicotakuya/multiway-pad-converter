# multiway-pad-converter

## overview

オープンソースによる、万能パッド変換器を自作するプロジェクト。

開発環境はArduino IDE。使用マイコンはArduino Pro Mini互換（ATmega328P,5V,16MHz）。

![chart](https://user-images.githubusercontent.com/5597377/174469409-51839756-9196-42b9-89b2-ea5085d715d5.png)

![multiway_pad_converter_beta](https://user-images.githubusercontent.com/5597377/174503133-65779209-de5d-49ed-879a-056d9470c409.jpg)

## 対応ハード

・メガドライブ 3ボタンパッド:

・メガドライブ 6ボタンパッド:

・メガドライブ アナログパッド:XE-1AP互換。左アナログスティックのXY軸で移動。右アナログスティックのY軸でスロットル。

・メガドライブ セガマウス:左アナログスティックのXY軸でカーソル移動。

・PCエンジン:

・PCエンジン アナログパッド:XE-1AP互換。左アナログスティックのXY軸で移動。右アナログスティックのY軸でスロットル。

・ファミコン:

・ファミコン アルカノイドコントローラ:左アナログスティックのX軸でバウスの移動。

・ファミコン アルカノイドII VS.mode:Port C0～1に可変抵抗、Port C2～4にボタンを追加することで、対戦モード時に2人同時プレイ。

・ファミコン クレイジークライマー:縦持ち状態のゲームパッド2個を再現。

・スーパーファミコン:

・スーパーファミコン マウス:左アナログスティックのXY軸でカーソル移動。

・X68000:

・X68000 アナログパッド:XE-1AP互換。左アナログスティックのXY軸で移動。右アナログスティックのY軸でスロットル。
 
・MSX アルカノイドコントローラ:


## 未対応。対応予定

・PCエンジン マウス:

## how to use

DualShock2の丸(〇)ボタンを押しっぱなしの状態で、電源オンまたはリセットすると、メニューが起動します。

選択したモード番号はEEPROMに保存されます。以後、保存したモードで起動します。

ソースコード内の"USE_CYBERSTICK"を"1"に変更することで、CYBERSTICKの入力、メガドライブ アナログパッドへの変換に対応。

## 回路図

"pad_conv_schematics.png" を参照。

## parts list

・Gamepad DualShock2

・U1:ATmega328P

・U2:74HC157 logic IC

・U3:3.3V voltage regulator

・X1:16MHz発振子

・OLED Display https://akizukidenshi.com/catalog/g/gP-12031/

・CN1:2x3 pin header

・CN2:playstation pad connecter

・CN3/4:Dsub9pin female

・CN5:mini DIN8pin male

・CN6:Dsub15pin female

・CN7:super famicom pad connecter

・CN8:4pin pin socket

・CN9:6pin pin header https://akizukidenshi.com/catalog/g/gM-11007/

・C1/3/4:0.1 micro F

・C2/5:100micro F 電解コンデンサ

・R1/2/3:1.5k ohm

・R4/5/6/7/8/9/10:3.3k ohm

・R11:10k ohm

・PS1:PS1:リセッタブルヒューズ(0.2A程度で切断を推奨) https://akizukidenshi.com/catalog/g/gP-12911/

・SW1:switch
