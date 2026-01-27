## 開発関係者
  * 寺田耕志

## ノード一覧
ノード名|概要
---|---
robot\_rrt\_planner\_node|プランニングのサービスを提供するノード

## 目的
* 全身関節角が与えられた際，あるいは目標リンク位置が与えられた際，干渉しない全身軌道を計算する．

## 使用例
  * `tmc_arm_palanning_tutorials`参照．

## 背景
* CBiRRT2による実装。手先位置等を含めたプランニングが可能。

## コンソールツール
  * コンソールでデバッグするコマンド．プランナがログフォルダに作成するファイルでプランニングをオフラインで再現できる．

    ツール名|概要
    ---|---
    plan\_by\_file|プランニングのリクエストファイルを元にオフラインでプランニングする．

## HSPインタフェース
  * 発行するトピック

    トピック名|型|概要
    ---|---|---
    ~debug\_joint\_state|sensor\_msgs/JointStates|計画中のJointStatesを発行
    ~debug\_environment|visualization\_msgs/MarkerArray|計画中の環境をrvizでデバッグするためのメッセージ

  * 提供するサービス

    サービス名|型|概要
    ---|---|---
    plan\_with\_constraints|tmc\_planning\_msgs/PlanWithConstraints|すべてをカスタムで与えたいときのサービス. CBiRRT2の全機能を使いたいとき
    plan\_with\_joint\_goals|tmc\_planning\_msgs/PlanWithJointGoals|単純なBiRRTとして利用する際のサービス
    plan\_with\_hand\_golas|tmc\_planning_msgs/PlanWithHandGoals|手先の最終位置を与えて計画する際のサービス
    plan\_with\_hand\_line|tmc\_planning_msgs/PlanWithHandLine|手先の直線軌道を与えて計画する際のサービス

      * メンバ詳細

        メソッド名|概要
        ---|---
        robot\_pose|ロボットの初期位置姿勢
        initial\_joint\_state|ロボットの初期関節角度 プランニングの対象にしない関節も含めて指示すること．
        use\_joints|プランニングの対象とする関節角名のリスト. IKがからむ場合は６つ以上手先までのルートに指定する必要がある．
        start\_joint\_states|初期関節角度のリスト　ここで指定した関節角度のうち干渉チェックを通ったものすべてを初期値とする．use\_jointsでの順番で与える
        start\_basejoint\_to\_bases|終端base位置のリスト　start\_joint\_statesに対応するbaseの位置姿勢．空にすると単位姿勢として解釈．
        goal\_joint\_states|終端関節角度のリスト　ここで指定した関節角度のうち干渉チェックを通ったものすべてを初期値とする．use\_jointsでの順番で与える
        goal\_basejoint\_to\_bases|終端base位置のリスト　goal\_joint\_statesに対応するbaseの位置姿勢．空にすると単位姿勢として解釈．
        start\_tsrs|初期TSRのリスト. このリストからランダムに初期値を追加．
        goal\_tsrs|終端TSRのリスト. このリストからランダムに終端値を追加．
        constraint\_tsrs|拘束を与えるTSR. リスト長さは０か１にすること．１以上を与えた場合は２つめ以降は無視される．
        probability\_start\_generate|初期関節角度をTSRからサンプリングする確率(0, 1)で指定
        probability\_goal\_generate|終端関節角度をTSRからサンプリングする確率(0, 1)で指定
        attached\_objects|ロボットが物体を持った際などに指定．ロボットの手先などの部位にその物体がついているとしてプランニングされる．
        hint\_trajectory|プランニングの際にヒントになる軌道．現在は使っていない
        environment\_before\_planning|プランニングの際の環境
        timeout|タイムアウト時間
        max\_iteration|プランニング最大繰り返し回数
        uniform\_bound\_sampling|trueにした場合初期姿勢および，終端姿勢の周辺の正規分布からTSRをサンプリングする．
        deviation\_for\_bound\_sampling|uniform\_bound\_samplingがtrueの際の標準偏差
        solution||解軌道
        base\_solution||ロボットbaseの解軌道
        environment\_after\_planning|運動計画後の環境
        joint\_state\_after\_planning|運動計画後の全関節角度
        origin\_to\_hand\_afeter\_planning|運動計画後の手先位置姿勢
        error\_code|エラーコード tmc\_manipulation\_msgs/ArmManipulationErrorCodes参照
        ref\_frame\_id|IKに使う参照フレーム．手先や指先など．
        origin\_to\_hand\_goals|基準座標から見たref\_frame\_idで指定したフレーム座標
        axis|基準座標系or手先座標で見た手先が動く軸方向
        local\_origin\_of\_axis true|手先で軸方向指定, false:基準座標で軸方向指定
        goal\_value|線の長さ[m]
        weighted_joints|重みをつける関節名のリスト．
        weight|重み．大きいほどその自由度をあまり使わずに逆運動学が計算される．TSRでゴールを探索する際の分布の幅もここで変更される．100.0くらいにするとほとんど動かない．特別な条件として、\_linear\_base、\_rotational\_baseでそれぞれ台車の並進と回転の重みを上書きできる.
        extra\_constraints|関節角間で連動したりする動作の拘束を与えるためのプラグインの名前．配列で複数指定できる．ここで指定した拘束は全般に渡って適応される．拘束のプラグインはtmc\_robot\_planner::IConfigurationConstraintから継承してください．
        extra\_start\_constraints|extra\_constraintsの初期姿勢にのみ適応させる拘束．
        extra\_goal\_constraints|extra\_constraintsの終端姿勢にのみ適応させる拘束．終端姿勢のみでものが見たいときなどに使う．
        start\_no\_ik\_joint\_state|初期姿勢をstart\_tsrsから決める際に，IKに関係ない首などの関節角度を指定する．
        goal\_no\_ik\_joint\_state|初期姿勢をgoal\_tsrsから決める際に，IKに関係ない首などの関節角度を指定する．

  * 利用するパラメータ

    パラメータ名|型|デフォルト値|概要
    ---|---|---|---
    robot\_description\_file|string||urdf形式のロボットモデルのファイルパス
    collision\_pair\_file|string||コリジョンチェック定義の設定ファイル
    ~delta|double|0.1|探索幅
    ~sub\_delta|double|0.1|干渉チェック幅
    ~publish\_debug\_info|bool|false|デバッグ情報をpublishするかのフラグ
    ~print\_debug\_info|bool|false|デバッグ情報をinfoで表示するかのフラグ
    ~save\_request|bool|false|デバッグ情報をログフォルダの下に保存する
    ~ik\_plugin|string|""|解析解のIKがあればプラグインとして追加(未実装)
    ~step\_mode|bool|false|ステップ実行をするモード.デバッグ時に便利
    ~collision\_engine|string|ODE|利用する干渉チェックエンジン
    ~weight_names|array(string)||プランナの探索幅や干渉チェック幅の重みをかける関節名のリスト．ここに書かれない関節はデフォルト値である1.0が使われる．
    ~weights|array(double)||プランナの探索幅や干渉チェック幅の重み．大きいとその自由度に関しては細かく探索や，干渉チェックし，最終的な軌道の刻みも小さくなる．`weight_names`と同サイズの配列にする必要あり．探索幅と出力幅に影響する．
    ~ik\_weight\_names|array(string)||逆運動学の重みをかける関節名のリスト．ここに書かれない関節はデフォルト値である1.0が使われる．
    ~ik\_weights|array(double)||逆運動学の自由度毎の重み．大きいほどその自由度をあまり使わずに逆運動学が計算される．，TSRでゴールを探索する際の分布の幅もここで変更される ．`ik_weight_names`と同サイズの配列にする必要あり．サービスのweightで上書きされるのもこちら．
    ~weight\_linear\_base|double|10.0|直線方向のbaseのプランナの探索幅や干渉チェック幅の重み．
    ~weight\_rotational\_base|double|10.0|回転方向のbaseのプランナの探索幅や干渉チェック幅の重み．
    ~weight\_linear\_base\_ik|double|10.0|直線方向のbaseのプランナの逆運動学の重み.
    ~weight\_rotational\_base\_ik|double|10.0|回転方向のbaseのプランナの逆運動学の重み.
    ~increase\_sampling\_deviation|boot|true|正規分布でゴールをサンプリングする際分散０から始めて，サービスで指定した分散まで増やしていくフラグ．
    ~step\_sampling\_deviation|double|0.01|increase_sampling_deviationがtrueの際に1stepあたり分散の増分．
    ~base\_translation\_max|double|10.0|並進方向の台車移動の最大値
    ~ik_plugins|array(string)||IKのプラグインの配列．後ろに書いたものから優先して使われる．

## テスト項目
ライブラリでプランニング自体のテストはしているので，単純なテストのみ．

デバッグ
======

デバッグのためのツールとして、再プランニング機能と結果再生機能がある。
これらを使うにはプランナの~save\_requestをTrueにしておく必要がある。
~save\_requestがtrueだとログフォルダ(__log指定, ROS_LOG_DIR, ROS_HOME/log, HOME/.ros/logの順)の下に

`plan_20130611T171139_110682175.request`

`plan_20130611T171139_110682175.response`

のようなファイルがプランニングを行う度に記録されていく。
ファイル名は日付と時間を合わせたものになっているので探す際の参考に
してほしい。
このうち*.requestはプランニングサービスのリクエストが保存され、
*.responseは解の軌道が保存されている。

再プランニング機能
------

まずrvizとデバッグモードにしたプランナを立ち上げる必要がある。
HSRなら

    $ roslaunch hsr_ant_manipulation planner_debug.launch

でrvizと合わせて、起動できるようにした。このあと、

    $ rosrun tmc_robot_rrt_planner_node plan_by_file ~/.ros/plan_xxxx.request

で以前の計画を再度実行できる．

結果再生機能
------

こちらは、以前の結果をそのまま表示できる機能である。
まずrvizとデバッグモードにしたプランナを立ち上げる必要がある。
HSRなら

    $ roslaunch hsr_ant_manipulation planner_debug.launch

でrvizと合わせて、起動できるようにした。このあと、

    $ rosrun tmc_robot_rrt_planner_node play_result ~/.ros/plan_xxxx.request ~/.ros/plan_xxxx.response

とする。ただしxxxは共通にすること．拡張子を除いて、requestとresponseを一度に与えることも可能.

    $ rosrun tmc_robot_rrt_planner_node play_result ~/.ros/plan_xxxx

* 利用するパラメータ

パラメータ名|型|デフォルト値|概要
---|---|---|---
~step\_mode|bool|false|ステップ実行するかしないか？
~wait_time|double|1.0|再生時のステップ単位でまつ時間[s]
