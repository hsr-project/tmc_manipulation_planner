`tmc_timeopt`
=============

概要
----
TOPP(Time Optimal Path Parametrization)問題の，ロボットやモデルによらないSolverを提供する．
- kinematicsとdynamicsを提供すれば計算できるSolver
- 速度・加速度リミットを提供すれば初速度込みで計算できるSolver

クラス
------

### kinematicsとdynamicsを提供すれば計算できるSolver

#### ユーザーが利用

- Timeopt: 最短時間制御問題を計算するクラス
- Target: 問題に合わせて実装する，TOPPの対象となるモデル定義クラス
- TrajectoryDict: 複数関節をまとめる，軌道管理用クラス
- Trajectory: pythonの辞書的に利用可能な軌道のインターフェイスクラス
    - LinearTrajectory: 線形補間
    - Poly3Trajectory: 3次多項式補間
    - Poly5Trajectory: 5次多項式補間
    - CubicSplineTrajectory: 3次スプライン補間
    - NaturalCubicSplineTrajectory: 自然3次スプライン補間

#### 内部実装

- kinematics.Kinematics: TOPPを解くためのKinematicsパラメータ計算
- dynamics.Dyamics: TOPPを解くためのDynamicsパラメータ計算
- poly2: 内部的に必要なる2次不等式の計算関数
- Interval: 複数の不等式で表される区間を表現

#### 使い方

example/demo_timeopt.py参照


### 速度・加速度リミットを提供すれば初速度込みで計算できるSolver

#### ユーザーが利用

- QuickTrajectoryFilter: 初速度込みの最短時間制御問題を計算するクラス

#### 内部実装
BSDライセンスで公開されているOSSを改変して利用．
ライセンス文はソースコード先頭に記載されている．
https://github.com/tobiaskunz/trajectories

- Trajectory: 最適化の実装部分，初速度を入力とするよう修正
- Path: 関節軌道を軌道長sで扱うための実装

#### 使い方

test/quick_trajectory_filter-test.cpp参照