# FIRA16_ws
Code for International FIRA robot competition

> Anyone willing to join the project development, please contact Erebuszz by daniel38437@gmail.com

## 注意事項
* 於 master 分支時，除了可做sync的動作外，不可有直接更動其上程式碼的行為
* 程式碼有任何值得紀錄的改變，都可以於自己開發的 branch 上執行 commit changes，有穩定的版本時再sync到branch的遠端函式庫
 (<b>記得每次都要檢查自己所在的 branch 是否正確</b>)
* 當有最終版本確定要上傳至 master 的遠端函式庫
 1. 移駕至 master 的當地函式庫與遠端作同步看看是否有何差異
 2. 回到自己的 branch 檢查是否有需要 Update from master；若有，則合併後檢查是否有衝突發生，
 至衝突發生處修改程式碼再做commit changes的動作；若沒有衝突但是有合併的動作，將合併後的程式碼實際執行確認過是否有問題需再做修正
 3. 以上完整確認過沒問題後，使用 pull request 向 master 要求合併程式碼，若沒有問題，網頁上顯示的 pull request 會顯示綠色打勾的狀態，
 便可自行在網頁上接受要求，確定同步
 4. 通知其他成員（包括自己）至 master 分支同步最新的程式碼，並合併到自己的分支
