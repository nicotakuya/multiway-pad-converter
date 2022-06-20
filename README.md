# multiway-pad-converter

## overview

DualShock2を使った万能パッド変換器。

開発環境はArduino IDE。使用マイコンはPro Mini互換（ATmega328P,5V,16MHz）。

![chart](https://user-images.githubusercontent.com/5597377/174469409-51839756-9196-42b9-89b2-ea5085d715d5.png)

![multiway_pad_converter_beta](https://user-images.githubusercontent.com/5597377/174503133-65779209-de5d-49ed-879a-056d9470c409.jpg)

## 対応ハード

・メガドライブ 3ボタンパッド

・メガドライブ 6ボタンパッド

・メガドライブ アナログ（XE-1AP互換）

・メガドライブ セガマウス

・PCエンジン

・PCエンジン アナログ（XE-1AP互換）

・ファミコン

・ファミコン アルカノイドコントローラ

・ファミコン クレイジークライマー

・スーパーファミコン

・スーパーファミコン マウス

・X68000

・X68000 アナログ（XE-1AP互換）
 
・MSX アルカノイドコントローラ


## 未対応。対応予定

・PCエンジン マウス

## how to use

DualShock2の丸(〇)ボタンを押しっぱなしの状態で、電源オンまたはリセットすると、メニューが起動します。

モード番号はEEPROMに保存されます。以後、選択したモードで起動します。

## 回路図

"pad_conv_schematics.png" を参照。

## parts list

・U1:ATmega328P

・U2:74HC157 logic IC

・U3:3.3V regulator

・X1:16MHz発振子

・OLED Display https://akizukidenshi.com/catalog/g/gP-12031/

・CN1:2x3 pin header

・CN2:playstation pad connecter

・CN3:Dsub9pin female

・CN4:Dsub9pin female

・CN5:mini DIN8pin male

・CN6:Dsub15pin female

・CN7:super famicom connecter

・CN8:4pin pin socket

・CN9:6pin pin header

・C1/3/4:0.1 micro F

・C2:100micro F

・R1/2/3:1.5k ohm

・R4/5/6/7/8/9/10:3.3k ohm

・R11:10k ohm

・PS1:

・SW1:
