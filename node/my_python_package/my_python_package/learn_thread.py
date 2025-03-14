import threading
import requests


class Download:
    def __init__(self):
        self.word_count = 0

    def download(self, url, callback_word_count):
        response = requests.get(url)
        content = response.text
        self.word_count = len(content.split())
        callback_word_count(self.word_count)

    def start_download(self, url, callback_word_count):
        thread = threading.Thread(target=self.download, args=(url, callback_word_count))
        thread.start()


def print_word_count(count):
    print(f"下载内容的单词数: {count}")


if __name__ == "__main__":
    url = "http://example.com"
    downloader = Download()
    downloader.start_download(url, print_word_count)
