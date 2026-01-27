`tmc_timeopt_ros`
==================

概要
----
TOPP(Time Optimal Path Parametrization)問題を解くROSノードを提供する
- TOPP問題のSolverは，tmc_timeoptが提供するものを利用
- 対応するロボットは，速度・加速度制約を持つ関節のリンクで構築されたロボットのみ


クラス
------

- SimpleJointTarget：　加速度拘束まで対応したTOPPの対象となるモデル定義
- TimeoptFilterNode：　TOPP問題を解くROSノード


インターフェイス
------------------

* 提供するサービス

- `filter_trajectory`(`tmc_manipulation_msgs/FilterJointTrajectory`)

   - Request
     - `trajectory`(`trajectory_msgs/JointTrajectory`):
       フィルタする軌道，time_from_startは空，poistionのみが意味を持つ
     - `start_states`(`tmc_manipulation_msgs/RobotState`): 初期状態
     - `limits`(`tmc_manipulation_msgs/JointLimits`): 各関節の，このリクエストのみで有効なリミット
     - `allowed_time`(`duration`): 許容時間[s]．利用していない

   - Response
     - `trajectory`(`trajectory_msgs/JointTrajectory`): フィルタされた軌道
     - `error_code`(`tmc_manipulation_msgs/ArmNavigationErrorCodes`): エラーコード，
     (SUCCESS=1: 成功, PLANNING_FAILED=-1: 失敗)のどちらかを返す

* パラメータ

  - `~(joint_name)/velocity` (double, default: 1.0)

      (joint_name)関節の最大速度．[rad/s] or [m/s]

  - `~(joint_name)/acceleration` (double, default: 1.0)

      (joint_name)関節の最大加速度．[rad/s^2] or [m/s^2]

  - `~minimum_dt` (double, default: 0.1)

     移動がない軌道が投げられた場合の再生時間 [s]

  - `~timeopt_resultion` (double, default: 0.2)

     最短時間制御の軌道長パラメータの分割幅． 無次元量．
     分割数をあげれば正確性が増すが計算量が増大する．
     (0.0, 1.0]の範囲にする必要がある．

  - `~velocity_ratio` (double, default: 1.0)
    速度のリミットに対する割合. dynamic_reconfigure対応. 0.1~1.0で指定．
    全ての関節の速度リミットをまとめて変更する．

  - `~acceleration_ratio` (double, default: 1.0)
    加速度のリミットに対する割合. dynamic_reconfigure対応. 0.1~1.0で指定．
    全ての関節の速度リミットをまとめて変更する．

