これは何？
=============================

サンプリングベースプランナのライブラリ．
現在は次の4つの問題を解くインターフェイスといくつかの実装を備える．
ただしMultiBiRRTがかなり汎用的な方法のため1,2,3はMultiBiRRTを使えば充分なことが多い．

開発関係者
================
・寺田耕志

目的
====
汎用なサンプリングベースのプランナを提供する．

設計指針
---------
std::functionを用いて，なるべく広範囲の問題に適応可能な用に設計している．
feasibility等も干渉チェックだけではなく，様々な条件を返すことが可能．

用語
-------------
* コンフィギュレーション: 計画問題で変数となる連続状態量．ロボットの関節角を並べたベクトルにすることが多い．
* feasibility : 実現性．マニピュレーションにおいては主にロボットが自分自身や環境と衝突していないかチェックする，

1. 一般遷移問題
--------------------
コンフィギュレーション空間上で，初期コンフィギュレーションと,終了条件を与えた際に
その間を遷移を与える．その間の軌道は有効であることが補償される．
終了条件は，範囲での指定や複数のコンフィギュレーションが利用可能．
(e.g. :　ロボットの複数把持姿勢への遷移，部品の3次元上での遷移)

- 実装
    - RRT
    - MultiBiRRT

2. 二点遷移問題
--------------------
コンフィギュレーション空間上で，初期コンフィギュレーションと終了コンフィギュレーションを与えた際，
その間を遷移を与える．その間の軌道は有効であることが補償される．
1.との違いは，終了コンフィギュレーションを明示的に与えることで高速な遷移が可能．

(e.g. :　ロボットの2姿勢遷移，部品の3次元上での遷移)

- 実装
     - BiRRT
     - MultiBiRRT


3. コンフィギュレーション最適化問題
--------------------
評価関数を与えると，最適でかつ可能なコンフィギュレーションを探す．
(e.g. : 干渉しない最適IK探索)

- 実装
    - モンテカルロ法

4. パスショートカット問題
------------------------
サンプリングベースで軌道計画を行うと，一般には長い軌道が計画されてしまう．
そこで軌道をつないでショートカットすることで，順最適な軌道を得る．

- 実装
    - ランダムショートカット
    - 総当たりショートカット
    - 2パス総当たりショートカット


クラス
==================

ConfigurationSpace
------------------
プランニングを行う状態空間を表現するクラス．ここに

* 状態空間の次元
* 状態空間上での距離
* どのような状態が許されるか(干渉チェック等)
* 状態の評価関数
* 状態空間の拘束
* スタートとゴールの生成方法
* ゴール状態の判定

を与える．
これらを与えることで，汎用的に利用可能なプランナを実現している．
これらはstd::fucntionで与えそれを実行している．
ConfigurationSpace構造体に与える関数は以下である．

* generate\_random\_config ランダム状態を返す関数．通常はコンフィギュレーション空間を一様にサンプルすればよい．
* check\_feasibility 現在のコンフィギュレーションが可能かを返す関数. 干渉チェックや姿勢拘束等．
* check\_transferability(Optional) ２つのコンフィギュレーションを直線的に遷移できるかチェック. 指定しない場合は始点と終点のみをcheck\_feasibilityでチェック．
* evalute\_conifg コンフィギュレーションの評価関数．
* check\_goal ゴール判定関数．
* generate\_goal\_config ゴール生成関数．
* calc\_distance(Optional) ２つのコンフィギュレーション間の距離. 指定しない場合はユーグリッド距離で計算される．
* constraint\_config コンフィギュレーションに拘束を与える．拘束が成功したらtrueを返し，失敗したらfalseを返す．
* constraint\_start\_config 初期コンフィギュレーションに拘束を与える．拘束が成功したらtrueを返し，失敗したらfalseを返す．
* constraint\_goal\_config 最終コンフィギュレーションに拘束を与える．拘束が成功したらtrueを返し，失敗したらfalseを返す．

拘束後のコンフィギュレーションは参照によって返す．
これらは必要なものだけ与えれば良い．

ConfigurationTree
------------------
状態空間中でのツリーを表現するクラス．RRT系のプランナで使う基本的な操作を含んでいる．
ConfigurationSpaceをメンバに持って動作する．以下のメソッドが定義されている．

* Extend ツリーの最近傍から与えた目標値までdeltaで枝を伸ばす．
* Connect Extendを目標値に到達するまで繰り返す．
* ClearTree ツリーを全消去．
* PrintTree Debug用．指定したストリームにツリーを出力する．
* TraceBackPath 最終要素から初期要素まで遡ってパスとして出力．
* GetLastConfig ツリーに最後に追加された状態を取得．
* RemoveLastBranch ツリーに最後に追加されたノードに連なるノードを消去．
* SetRootConfig ツリーの初期要素を追加．
* GetNumNode ツリーのノード数を取得．

IPointToConditionPlanner
------------------------
コンフィギュレーションからゴール状態までの計画を行うインターフェイス，ゴール状態は範囲等で記述可能．

RRTPlanner
----------
RRTを用いたIPointToConditionPlannerの実装．

IPointToPointPlanner
------------------------
2コンフィギュレーション間をつなぐ計画を行うインターフェイス．

BiRRTPlanner
------------
BiRRTを用いたIPointToPointPlannerの実装．

IMultiPlanner
--------------
多数のゴールを用いた計画を行うインターフェイス．

IMultiRrtPlanner
--------------
多数のゴールを用いた計画を行うRRTの実装．スタート，ゴールの追加を含む．

IConfigOptimizer
----------------
ある値を最適化するインターフェイス．

RandomOptimizer
---------------
単純なモンテカルロ法による最適化．

IPathShortCutter
----------------
サンプリングベースのプランナから出てきた荒いパスをスムーズにする機能のインターフェイス．

RoundRobinShortCutter
--------------------
総当たりにによるショートカット．


テスト仕様
==========

ConfigurationSpaceのテスト
--------------------------

* CheckTransferabilityByDibidingTest::normal\_test コンフィギュレーション[0.0, 0.0]と[1.0, 1.0]を0.1で再分割してチェックできるかテスト．
始点と終点は許されるが，中間が許されない状態を正しく判別できるか調べる．

* CheckTransferabilityByDividingTest::dim\_mismatch 始点と終点のコンフィギュレーションの次元が違うときに例外を送出するかチェック．

* CheckTransferabilityByDividingTest::negative\_subdelta 再分割幅が負であると例外送出するかチェック

* TreeToPathTest::translate\_path\_last Treeをパスに変換するテスト．ゴールがツリーの最終要素の場合.

* TreeToPathTest::translate\_path\_mid Treeをパスに変換するテスト．ゴールがツリーの中間要素の場合.

* TreeToPathTest::translate\_path\_loop Treeにループがあった場合TreeToPathがTreeLoop例外を投げて終了するかチェック．

* TreeToPathTest::translate\_path\_invalid\_goal TreeToPathで存在しないゴールを指定された場合invalid\_argument例外を投げて終了するかチェック．

* ConfigurationSpaceTest::new\_config new\_config new\_configでツリーを正しく拡張できるかチェック．

* ConfigurationSpaceTest::random\_config 設定したrandom\_configが正しく呼ばれるかチェック．

* ConfigurationSpaceTest::check_line CheckLineで設定が悪いため失敗するテスト

* ConfigurationSpaceTestNoFunctions::no\_random\_config\_func 必要な関数が設定されていない例外のチェック．

* ConfigurationSpaceTestNoConstrain::check\_line 2点間を結んでfeasibleであるかチェック．

* ConfigurationSpaceTest::check\_feasibility Feasibilityのチェックが働くかチェック．

* ConfigurationSpaceTestNoFunctions, no\_check\_feasibility\_func CheckFeasibilityが設定されてない場合の例外チェック

* ConfigurationSpaceTest::check\_transferability check\_transferabilityが働くかチェック．

* ConfigurationSpaceTestNoFunctions::no\_check\_transferability\_func CheckTransferabilityが設定されていなくても遷移チェックが働くかのチェック．

* ConfigurationSpaceTest::calc\_distance 距離の計測が設定されたものでちゃんと動くか調べる．

* ConfigurationSpaceTestNoFunctions::no\_calc\_distance\_func CalcDistanceが設定されていなかったらユーグリッド距離で計算されるかチェック．

* ConfigurationSpaceTest::eval\_func 評価関数が働くかチェック．

* ConfigurationSpaceTestNoFunctions::no\_evaluate\_config\_func Evaluate関数が設定されていないので例外を投げるかチェック．

* ConfigurationSpaceTest::goal\_func goalコンフィギュレーションの生成チェック．

* ConfigurationSpaceTest::no\_generate\_goal\_config\_func goalコンフィギュレーションの生成関数がない例外チェック．

* ConfigurationSpaceTest::start\_func startコンフィギュレーションの生成チェック．

* ConfigurationSpaceTest::no\_generate\_start\_config\_func startコンフィギュレーションの生成関数がない例外チェック．

* ConfigurationSpaceTest::constraint\_config\_func コンフィギュレーションを拘束ができるかチェック

* ConfigurationSpaceTestNoFunctions::no\_constraint\_config\_func 拘束がない場合のチェック

ConfigurationTreeのテスト
--------------------------

* ConfigurationTreeTest::Extend ツリーの1-step拡張が正しく行われるかテスト．結果のReached,Traped,Advancedが正しいかチェック

* ConfigurationTreeTest::Connect ツリーの拡張が正しく行われるかテスト．

RrtPlannerのテスト
-----------------

* RrtPlannerTest::plan プランニングが成功するだろうケースで成功するかテスト.

* RrtPlannerTest::max\_itr 最大繰り返し回数でチェックが終わるテスト.

* RrtPlannerTest::terminate 設定した終了条件で計画が終わるかテスト．

BiRrtPlannerのテスト
-----------------

* BiRrtPlannerTest::plan プランニングが成功するだろうケースで成功するかテスト.

* BiRrtPlannerTest::max\_itr 最大繰り返し回数でチェックが終わるテスト.

* BiRrtPlannerTest::terminate 設定した終了条件で計画が終わるかテスト．

MultiBiRrtPlannerのテスト
------------------------

* MultiBirrtPlannerTest::plan プランニングが成功するだろうケースで成功するかテスト.

* MultiBirrtPlannerTest::max\_itr 最大繰り返し回数でチェックが終わるテスト.

* MultiBirrtPlannerTest::terminate 設定した終了条件で計画が終わるかテスト．

* MultiBirrtPlannerTest::multi\_goal\_test 複数ゴールに対応しているかテスト.

* MultiBirrtPlannerTest::generate\_test 初期値やゴールを生成してテスト

* MultiBirrtPlannerTest::init\_config\_fail 初期値での失敗テスト

* MultiBirrtPlannerTest::goal\_config\_fail 終端値での失敗テスト

RandomOptimizerのテスト
-----------------------
* RaundRobinShortCutterTest::simple\_shortcut 実際にパスが短くなるかをテスト．

* RaundRobinShortCutterTest::short\_cut\_empty\_exceptional 空のぱすに対して例外を投げるかチェック．

* RaundRobinShortCutterTest::short\_cut\_empty\_exceptional 空のぱすに対して例外を投げるかチェック．

* RaundRobinShortCutterTest::short\_cut\_negative\_skip\_exceptional 負のスキップ幅に対しての例外

* RaundRobinShortCutterTest::terminate 終了条件関数で終了するかチェック．

RandomOptimizerのテスト
----------------------
* RandomOptimizerCheck::optim\_check 簡単な関数がほぼ最適になるかチェック．

使い方（2点遷移問題）
===================
ソースはexample/birrt2d.cppにもある
2次元の遷移問題の例を示す．ここでは下図の点[0,0]，から[4.0,4.0]への遷移を解く．

![問題設定](problem.png "問題設定")


** ソースコード **
~~~~
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <point_to_point_planner.hpp>
#include <birrt_planner.hpp>
#include <path_short_cutter.hpp>
#include <raund_robin_short_cutter.hpp>

using namespace tmc_rplanner;

const int32_t kDim(2);
const double kDelta(0.1);

static double Randd()
{
    return (double)rand()/RAND_MAX;
}

Config RandomConfig(){
  Config v(2);
  v(0) = Randd()*4.0;
  v(1) = Randd()*4.0;
  return v;
}

bool CollisionCheck(const Config& config){
  if (((config(0) < 3.5) && (config(0)>0)) && ((config(1) < 1.5) && (config(1)>1.0))) return false;
  if (((config(0) < 4.0) && (config(0)>0.5)) && ((config(1) < 3.5) && (config(1)>3.0))) return false;
    return true;
}



int main(int argc,char* argv[])
{
  Config init(kDim);
  init << 0.0,0.0;
  Config goal(kDim);
  goal << 4.0,4.0;

  ConfigurationSpacePtr space(new ConfigurationSpace(kDim));
  space->set_random_config(RandomConfig);
  space->set_check_feasibility(CollisionCheck);

  IPointToPointPlannerPtr planner(new BiRrtPlanner(space, kDelta, 10000));

  Path path;
  Path opt_path;
  double length = 0.0;
  if (planner->PlanPath(init,goal,path) == kSuccess) {
    IPathShortCutterPtr short_cutter(new RoundRobinShortCutter(space, kDelta, true, 1));
    short_cutter->ShortCut(path,opt_path);
    for (size_t i = 0; i < opt_path.size()-1; ++i) {
      std::cout << opt_path[i].transpose() << " " << opt_path[i+1].transpose() << std::endl;
      length += space->CalcDistance(opt_path[i], opt_path[i+1]);
    }
  }

  return 0;
}


~~~~

- 空間サンプル関数を定義する．ここでは単純に[0,4.0],[0,4.0]の乱数を発生させる．

~~~~~~
Config RandomConfig(){
  Config v(2);
  v(0) = Randd()*4.0;
  v(1) = Randd()*4.0;
  return v;
}

~~~~~~

- 干渉チェック関数を定義する．可能なコンフィギュレーションに対して，true,不可能なコンフィギュレーションに対して，
falseを返す関数を定義．
~~~~~~
bool CollisionCheck(const Eigen::VectorXd& config){
  if (((config(0) < 3.5) && (config(0)>0)) && ((config(1) < 1.5) && (config(1)>1.0))) return false;
  if (((config(0) < 4.0) && (config(0)>0.5)) && ((config(1) < 3.5) && (config(1)>3.0))) return false;
  return true;
}

~~~~~~

- コンフィギュレーション空間に次元を与えて定義．

~~~~~~
const int32_t kDim(2);
const double kDelta(0.1);
:
ConfigurationSpacePtr space(new ConfigurationSpace(kDim));

~~~~~~

- コンフィギュレーション空間の関数を設定．

~~~~~~
space->set_random_config(RandomConfig);
space->set_check_feasibility(CollisionCheck);

~~~~~~

- プランナを作成　最大繰り返し回数は10000回に設定

~~~~~~
IPointToPointPlannerPtr planner(new BiRrtPlanner(space, kDelta, 10000));
~~~~~~

- プランニング

~~~~~~
if (planner->PlanPath(init,goal,path) == kSuccess) {
~~~~~~

- ショートカット そのままのパスは非常に効率が悪いのでショートカット．
ここでは2-パスの総当たりショートカットを選択

~~~~~~
IPathShortCutterPtr short_cutter(new RoundRobinShortCutter(space, kDelta, true, 1));
short_cutter->ShortCut(path,opt_path);
~~~~~~

** 結果 **

![結果](result.png "結果")

