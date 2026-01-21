Overview
++++++++

提供機能
---------
* CBiRRT2による干渉しない全身軌道を計画する機能
* 全身軌道計画のログから再計画を行う機能
* 全身軌道計画のログを再生し、可視化する機能

ROS Interface
++++++++++++++

robot_rrt_planner_node
-----------------------
CBiRRT2による干渉しない全身軌道を計画するサービスを提供するノードです。

Published Topics
^^^^^^^^^^^^^^^^

- **debug_joint_state** (:ros:msg:`sensor_msgs/JointState`) 全身軌道計画途中のロボット関節姿勢、デバッグ用
- **debug_environment_pub_** (:ros:msg:`visualization_msgs/MarkerArray`) 全身軌道計画途中の環境情報、デバッグ用

Service Server
^^^^^^^^^^^^^^

- **plan_with_constraints** (:ros:srv:`tmc_planning_msgs/PlanWithConstraints`) CBiRRT2の全機能を使用する全身軌道計画
- **plan_with_joint_goals** (:ros:srv:`tmc_planning_msgs/PlanWithJointGoals`) ロボット関節姿勢を目標とする全身軌道計画
- **plan_with_hand_goals** (:ros:srv:`tmc_planning_msgs/PlanWithHandGoals`) 手先位置・姿勢を目標とする全身軌道計画
- **plan_with_hand_line** (:ros:srv:`tmc_planning_msgs/PlanWithHandLine`) 手先の直線軌道を導出する全身軌道計画

Parameter
^^^^^^^^^

- **robot_description** (string) urdf形式のロボットモデル
- **robot_collision_pair** (string) xml形式の干渉検出設定
- **~delta** (double: 0.1) 探索空間での探索幅
- **~sub_delta** (double: 0.02) 探索空間での干渉チェック幅
- **~publish_debug_info** (bool: false) デバッグ情報をpublishするかのフラグ
- **~print_debug_info** (bool: false) デバッグ情報をROS_INFOで表示するかのフラグ
- **~save_request** (bool: false) デバッグ情報をログフォルダの下に保存するかのフラグ
- **~step_mode** (bool: false) 全身軌道計画をステップ実行するかのフラグ
- **~base_translation_max** (double: 10.0) 並進方向の台車移動の最大値[m] 
- **~ik_plugins** (string: []) IKのプラグインの配列、後ろに書かれているほど優先度高
- **~weight_names** (string[]: []) 探索空間で重み付けされる関節名
- **~weights** (double[]: []) 探索空間での関節ごとの重み
- **~weight_linear_base** (double: 10.0) 探索空間での台車並進移動の重み 
- **~weight_rotational_base** (double: 10.0) 探索空間での台車回転移動の重み 
- **~ik_weight_names** (string[]: []) IK時の重み付けされる関節名
- **~ik_weights** (double[]: []) IK時の関節ごとの重み
- **~weight_linear_base_ik** (double: 10.0) IK時の台車並進移動の重み 
- **~weight_rotational_base_ik** (double: 10.0) IK時の台車回転移動の重み 
- **~collision_engine** (string: ODE) 利用する干渉チェックエンジン

play_result
------------
全身軌道計画のログを再生し、可視化するスクリプトです。デバッグ用途を想定しています。

Published Topics
^^^^^^^^^^^^^^^^

- **debug_joint_state** (:ros:msg:`sensor_msgs/JointState`) 再生されたロボット関節姿勢
- **debug_environment_pub_** (:ros:msg:`visualization_msgs/MarkerArray`) 再生された環境情報

Parameter
^^^^^^^^^

- **robot_description** (string) urdf形式のロボットモデル
- **robot_collision_pair** (string) xml形式の干渉検出設定
- **collision_engine** (string: ODE) 利用する干渉チェックエンジン
- **~step_mode** (bool: false) 再生をステップ実行するかのフラグ
- **~wait_time** (double: 0.2) ステップ実行しない場合の全身軌道の１点を表示する時間[sec]

How to use
++++++++++

全身軌道計画のデバッグ
-----------------------
全身軌道計画が失敗した時、全身軌道計画の結果が思い通りでないときのデバッグ方法を説明します。
urdf形式のあらゆるロボットモデルに対応していますが、ここではHSRBを例として使用します。

ログの取得
^^^^^^^^^^^
`robot_rrt_planner_node` はパラメータ `~save_request` がTrueの場合、
全身軌道計画サービスのログとして、リクエストとレスポンス（計画成功時のみ）を出力します。
出力先はログフォルダ(ROS_LOG_DIR, ROS_HOME/log, HOME/.ros/logのうち存在するもの、左ほど優先度高)です。
HSR-Bの実機、シミュレータともに、 `~save_request` はTrueになっています。

まずは、全身軌道計画サービスのログを作成するためにシミュレータを起動します。

.. code-block:: bash

   $ roslaunch hsrb_gazebo_launch hsrb_hcr2013_world.launch

軌道時にターミナルに表示される情報の中にログフォルダが含まれていますので、それを覚えておいて下さい。
次のように表示された場合、ログフォルダは/home/toyota/.ros/log/b306598c-f795-11e5-b9f6-a0481c8bc5d8となります。

.. code-block:: bash

   ... logging to /home/toyota/.ros/log/b306598c-f795-11e5-b9f6-a0481c8bc5d8/roslaunch-toyota-3708.log


次に、ihsrbを起動します。

.. code-block:: bash

   $ ihsrb

ihsrbの中で全身軌道計画を利用する関数を呼び出します。

.. code-block:: python

   In [1]: whole_body.move_to_go()

シミュレータとihsrbを終了し、出力されたファイルを確認します。
ログフォルダはシミュレータ立ち上げ時に覚えておいたフォルダを利用して下さい。

.. code-block:: bash

   $ cd ${ログフォルダ}
   $ ls

すると、 `plan_20160401T074828_154000000.request` 、
`plan_20160401T074828_154000000.response` のようなファイルが見つかるはずです。
これらが、全身軌道計画サービスのログです。
ファイル名は日付と時間を合わせたものになっており、デバッグしたいファイルを探す際の参考になります。

ログの再計画
^^^^^^^^^^^^^
まずは、デバッグ用の全身軌道計画ノードを起動します。

.. code-block:: bash

   $ roslaunch hsrb_manipulation_launch hsrb_planner_debug.launch

次に、再計画を実行します。再計画の対象となるのは、 `plan_xxxx.request` のようなファイルです。
ここでは `plan_20160401T074828_154000000.request` を対象としていますが、適時書き換えて下さい。

.. code-block:: bash

   $ rosrun tmc_robot_rrt_planner_node plan_by_file plan_20160401T074828_154000000.request

ログの表示
^^^^^^^^^^^
再計画と同様に全身軌道計画ノードを起動します。

.. code-block:: bash

   $ roslaunch hsrb_manipulation_launch hsrb_planner_debug.launch

次に、ログの表示を実行します。表示するためには、
`plan_xxxx.request` と `plan_xxxx.response` が揃っている必要があります。
ここでは `plan_20160401T074828_154000000.request`　と `plan_20160401T074828_154000000.response`
を対象としていますが、適時書き換えて下さい。

.. code-block:: bash

   $ rosrun tmc_robot_rrt_planner_node play_result plan_20160401T074828_154000000

全身軌道計画ノードと同時に起動したrvizに動作が表示されます。
