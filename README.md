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
## 🎬 シミュレーション実行結果 / 動画 (Video Demos)

本プロジェクトにおける各プログラムの実行結果動画（動作デモ）です。

---

### 1. メインシミュレーション（LQR動的制御・外乱＆軸受摩擦あり）
* **実行ファイル:** `simulation_control_scipy.py`
* **特徴:** 1.25秒周期のパルス外乱（手で突くような衝撃）および回転軸の粘性摩擦（$b_{\text{pole}}=0.05$）が存在する環境下で、LQR制御によって即座に姿勢を立て直す挙動。

<!-- ==================== 👇 【動画1の挿入位置】 👇 ==================== -->
<!-- GitHubの編集画面で、下の行に動画(.mp4/.mov)またはGIFを直接ドラッグ＆ドロップしてください -->


https://github.com/user-attachments/assets/67c0a739-8858-4351-9734-c5fed8642d97





<!-- ==================== 👆 【動画1の挿入位置】 👆 ==================== -->

---

### 2. オフライン計算ゲインによる基本 LQR 制御
* **実行ファイル:** `simulation_control_OfflineCalculation.py`
* **特徴:** 事前計算された固定ゲイン $\mathbf{K}$ を用いて、初期偏角（-30°）から振子をスムーズに直立平衡点へ収束させる基本動作。

<!-- ==================== 👇 【動画2の挿入位置】 👇 ==================== -->
<!-- GitHubの編集画面で、下の行に動画(.mp4/.mov)またはGIFを直接ドラッグ＆ドロップしてください -->

https://github.com/user-attachments/assets/45953273-ba9b-402d-805d-e1b749260c8a


<!-- ==================== 👆 【動画2の挿入位置】 👆 ==================== -->

---

### 3. 制御なし（自由落下・自然挙動）の比較検証
* **実行ファイル:** `simulation_nocontrol.py`
* **特徴:** 制御入力なし（$u = 0$）の状態。初期偏角（60°）から重力に従って振子が倒れていく自然挙動（LQR制御効果の比較用）。

<!-- ==================== 👇 【動画3の挿入位置】 👇 ==================== -->
<!-- GitHubの編集画面で、下の行に動画(.mp4/.mov)またはGIFを直接ドラッグ＆ドロップしてください -->

https://github.com/user-attachments/assets/3f3cc471-92c7-46ed-9c49-907f7eb65d4d


<!-- ==================== 👆 【動画3の挿入位置】 👆 ==================== -->

---
