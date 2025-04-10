# ベースイメージを指定
FROM nvidia/cuda:11.8.0-base-ubuntu22.04

# 作業ディレクトリを設定
WORKDIR /code

# 必要なパッケージのインストールと設定
RUN apt-get update && \
    apt-get -y upgrade && \
    apt-get install -y curl python3 python3-distutils git && \
    curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python3 get-pip.py && \
    rm get-pip.py

# OpenCVの依存関係のインストール
RUN apt-get update && apt-get install -y ffmpeg libsm6 libxext6
# - ffmpeg: 動画処理のため
# - libsm6, libxext6: OpenCVの依存関係

# requirements.txtをコピーし、必要なPythonパッケージをインストール
COPY ./requirements.txt /code/
RUN pip install --upgrade pip && \
    pip install -r requirements.txt

# Gitの設定
RUN git config --global user.email "hoge.hoge@gmail.com" && \
    git config --global user.name "hoge-hoge"

# データストアとデータセット用のボリュームを設定
VOLUME /mnt/nfs_p

# 作業ディレクトリを設定
WORKDIR /mnt

# fstabファイルのコピー
COPY ./fstab /etc/

# Jupyter Notebookの設定ファイルをコピー
COPY jupyter_notebook_config.py /root/.jupyter/jupyter_notebook_config.py

# デフォルトコマンドを設定
CMD mount -a & jupyter lab --ip=0.0.0.0 --allow-root --port 8888 --NotebookApp.token='' --no-browser