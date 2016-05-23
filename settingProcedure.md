# 實測機器人設定程序
> <b>每開啟一個分頁必須先於專案目錄內使用指令：source devel/setup.bash</b>

1. $ roscore
2. $ rosparam load ~/FIRA16_ws/src/vision/prosilica_driver /prosilica_driver
  * 讀取攝影機的參數設定
3. $ rosrun prosilica_camera prosilica_node
4. $ rosrun vision interface
  * 開啟介面設定色模、距離模...等，每個區塊設定完記得都要按 sent 及 save 的按鍵
5. $ rosrun vision objectdetection
  * 執行物件分割使機器人能夠辨別各物件之位置
6. ☑view
  * 於第四點內開啟的介面中勾選，用於顯示物件分割後之球及球門位置
7. $ rosrun strategy FIRA_strategy
  * 執行機器人策略
8. $ sudo chmod 777 /dev/ttyUSB0
9. $ rosrun motion_test motion_test
10. $ rostopic pub /FIRA/GameState std_msgs/Int32 1<br>
    $ rostopic pub /FIRA/TeamColor std_msgs/String "Blue"
  * 設定此機器人為<b>藍隊</b>並處於<b>attack</b>的遊戲狀態
