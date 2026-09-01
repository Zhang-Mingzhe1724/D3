# 倒立振子 (Cart-Pole) LQR 最適制御物理シミュレーション

本リポジトリは、倒立振子（Cart-Pole）システムの非線形物理モデル構築、一階泰勒展開による線形化、および代数リカッチ方程式（ARE）を用いた LQR（線形2次レギュレータ）最適制御のシミュレーションコード一式です。

---

## 📁 ファイル構成と役割

| ファイル名 | 概要・機能の説明 |
| :--- | :--- |
| **`simulation_control_scipy.py`** | **【メイン実行ファイル】** 代数リカッチ方程式（ARE）による LQR 動的制御、軸受粘性摩擦、1.25秒周期のパルス外乱を含む物理シミュレーション[cite: 11]。 |
| **`scipy_test.py`** | 状態空間行列 $\mathbf{A}, \mathbf{B}$ を構築し、ARE を解いて LQR フィードバックゲイン $\mathbf{K}$ を算出・表示する計算スクリプト[cite: 9]。 |
| **`simulation_control_OfflineCalculation.py`** | 算定済みの固定ゲイン $\mathbf{K}$（オフライン計算値）を直接使用して駆動する基本 LQR 制御シミュレーション[cite: 10]。 |
| **`simulation_nocontrol.py`** | 制御入力なし ($u=0$) の状態における振子の自然落下・自由揺晃動作（比較検証用）[cite: 12]。 |
| **`import_test.py`** | 実行環境（NumPy, Matplotlib）の正常動作を確認するためのテストスクリプト[cite: 8]。 |

---

## 🧮 制御理論の概要 (LQR Control)

### 1. 状態空間モデルの構築と線形化
台車の位置 $x$、速度 $\dot{x}$、振子の偏角 $\theta$、角速度 $\dot{\theta}$ を状態向量 $\mathbf{x} = [x, \dot{x}, \theta, \dot{\theta}]^T$ と定義し、直立平衡点（$\theta \approx 0$）の周りで泰勒展開（小角度近似：$\sin\theta \approx \theta, \cos\theta \approx 1$）を行い、状態方程式を構築します。

$$\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}u$$

### 2. 代数リカッチ方程式 (ARE) の解法
評価関数（Cost Function）$J$ を最小化します：

$$J = \int_{0}^{\infty} \left( \mathbf{x}^T \mathbf{Q} \mathbf{x} + u^T \mathbf{R} u \right) dt$$

以下の代数リカッチ方程式（ARE）を解き、行列 $\mathbf{P}$ を算出します：

$$\mathbf{A}^T \mathbf{P} + \mathbf{P}\mathbf{A} - \mathbf{P}\mathbf{B}\mathbf{R}^{-1}\mathbf{B}^T\mathbf{P} + \mathbf{Q} = 0$$

### 3. 最適フィードバック制御力の算出
$$\mathbf{K} = \mathbf{R}^{-1}\mathbf{B}^T\mathbf{P}$$
$$u = -\mathbf{K}\mathbf{x} = -\left( k_x x + k_{\dot{x}} \dot{x} + k_\theta \theta + k_{\dot{\theta}} \dot{\theta} \right)$$

---

## ⚙️ 動作環境 (Requirements)

* Python 3.x
* NumPy
* SciPy
* Matplotlib

```bash
pip install numpy scipy matplotlib
```


## 🚀 実行方法 (How to Run)

### 1. メインシミュレーションの実行（外乱・摩擦あり）
```bash
python simulation_control_scipy_4.py<br>
```
※ 1.25秒周期の外乱衝撃と回転軸摩擦のもとで、LQR制御が振子を立て直すアニメーションが表示されます。

### 2. LQR フィードバックゲイン $K$ の単体計算
```bash
python scipy_test.py
```
### 3. オフライン計算ゲインによる基本 LQR 制御の実行
```bash
python simulation_control_OfflineCalculation.py
```
### 4. 制御なし（自由落下）の挙動確認
```bash
python simulation_no_control.py
```

