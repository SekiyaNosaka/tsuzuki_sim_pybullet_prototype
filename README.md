# tsuzuki_sim_pybullet_prototype

## これは何？
UR5/Robotiq 2F-85によるpybulletを用いたビンピッキングシミュレータ

  > これはプロトタイプ版

## 動作確認環境
- OS: Ubuntu 18.04, 20.04
- Python 3.6, 3.8
  - pybullet: 3.2.5

## インストール
- 依存ライブラリのインストール

```term=
$ pip install pybullet
$ (or pip3 install pybullet)
$ pip install tqdm
$ (or pip3 install tqdm)
```

  > 基本はこれで大丈夫だと思うが，依存ライブラリが不足していたら，適宜対処

- 本パッケージのインストール

```term=
$ git clone https://github.com/SekiyaNosaka/tsuzuki_sim_pybullet_prototype.git
```

## 実行方法

```term=
$ cd scripts
$ python main.py
$ (or python3 main.py)
```

<img src="https://i.imgur.com/FYxnNlA.png" width="800">
<img src="https://i.imgur.com/VCAsCSw.png" scale="600">``
