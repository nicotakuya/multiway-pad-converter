# Multiway pad converter

## Overview

万能パッド変換器を自作するプロジェクト。

開発環境はArduino IDE。使用マイコンはArduino Pro Mini相当（ATmega328P,5V,16MHz）。

![chart](https://user-images.githubusercontent.com/5597377/174469409-51839756-9196-42b9-89b2-ea5085d715d5.png)

![multiway_pad_converter_beta](https://user-images.githubusercontent.com/5597377/174503133-65779209-de5d-49ed-879a-056d9470c409.jpg)

## MENU

・MDRIVE-3BUTTON:メガドライブ 3ボタンパッド。Dpad→移動。START→START。四角／バツ／マル→A/B/Cに割り当て。

・MDDRIVE-6BUTTON:メガドライブ 6ボタンパッド。Dpad→移動。SELECT/START→MODE/START。四角／バツ／マル→A/B/C。三角/L1/R1/→X/Y/Zに割り当て。（＊認識しない場合は本体リセット）

・MDDRIVE-ANALOG:メガドライブ アナログパッド。XE-1AP互換。左アナログスティックのXY軸で移動。右アナログスティックのY軸でスロットル。（＊起動後に本体リセットしないと認識できない）

・MDDRIVE-MOUSE:メガドライブ セガマウス:左アナログスティックのXY軸でカーソル移動。「雀皇登竜門」で確認。

・PCENG-DIGITAL:PCエンジン デジタル。Dpad→移動。マル／バツ→I/IIに割り当て。

・PCENG-ANALOG:PCエンジン アナログパッド。XE-1AP/CYBERSTICK互換。左アナログスティックのXY軸で移動。右アナログスティックのY軸でスロットル。

・PCENG-MOUSE:PCエンジン マウス。左アナログスティックのXY軸でカーソル移動。マル／バツ→右クリック/左クリックに割り当て。（＊起動後に本体リセットしないと認識できない）

・FAMI-DIGITAL:ファミコン。Dpad→移動。SELECT/START→SELECT/START。バツ／マル→B/A。

・FAMI-PADDLE:ファミコン アルカノイド パドルコントローラ。左アナログスティックのX軸でバウスの移動。

・FAMI-PADDLE-VS.:ファミコン アルカノイドII VS.mode:Port C0～1に可変抵抗、Port C2～4にボタンを追加することで、対戦モード時に2人同時プレイ。

・FAMI-CRAZYCL:ファミコン クレイジークライマー。縦持ち状態のゲームパッド2個を再現。Dpad→左手。四角／三角／マル／バツ→右手。

・SFC-DIGITAL:スーパーファミコン。Dpad→移動。SELECT/START→SELECT/START。バツ／マル→B/A。四角／三角→X/Y。L1／R1→L/R。

・SFC-MOUSE:スーパーファミコン マウス。左アナログスティックのXY軸でカーソル移動。

・X68K-DIGITAL:X68000。Dpad→移動。バツ／マル→B/A。

・X68K-ANALOG:X68000 アナログパッド。XE-1AP/CYBERSTICK互換。左アナログスティックのXY軸で移動。右アナログスティックのY軸でスロットル。

・MSX-PADDLE:MSX アルカノイド パドルコントローラ。左アナログスティックのX軸でバウスの移動。

・MSX-PADDLE-VOL:MSX アルカノイド パドルコントローラのボリューム版。Port C0に可変抵抗、Port C2にボタンを追加することで、ボリューム操作が可能。


## how to use

DualShock2のいずれかのボタンを押しっぱなしの状態で、電源オンまたはリセットすると、メニューが起動します。

選択したモード番号はEEPROMに保存されます。以後、保存したモードで起動します。

Dsub9pinコネクタを追加して、ソースコード内の"USE_CYBERSTICK"を"1"に変更することで、CYBERSTICK→メガドライブ アナログパッドへの変換に対応。

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

・SW1/SW2/SW3:tact switch
