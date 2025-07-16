import threading        # 多线程 库
import requests         # 下载 库

'''
    项目：下载 小说，并统计字数
'''
class Download:
    def download(self,url,callback_word_count):             # callback_word_count 为 回调函数(为形参)，用于 统计字数
        # pass                                                # 表示 内部是空的，什么都没有
        # 打印 线程编号，方便看在哪个线程里.threading.get_ident()获取线程编号
        print(f'线程：{threading.get_ident()} 开始下载：{url}')     
        # 请求，并 下载
        response = requests.get(url)      
        # 下载后的结果 , 使用 utf-8 的编码
        response.encoding = 'utf-8'
        # 调用 回调函数,进行 字数的统计
        callback_word_count(url,response.text)               # 参数：地址url 与 内容result



    def start_download(self,url,callback_word_count):       # 开始 下载 函数
        
        # 需 启动线程 去 调用 download这个函数 完成下载 与 回调
        
        # 创建对象，创建一个新的线程。 target 线程启动后执行的目标函数, args 为 目标函数的形参
        thread = threading.Thread(target=self.download,args=(url,callback_word_count))          # 创建好 线程了
        # 启动线程
        thread.start()                  



'''
    函数： 普通函数，用于回调
'''
def word_count(url,result):      # 计算 字体数量 的函数
    print(f"{url}:{len(result)}->{result[:5]}")             # [] 为 切片参数


def main():
    # 创建对象
    download = Download()
    # 下载 三个网址内容，使用 三个线程进行下载
    download.start_download('http://0.0.0.0:8000/novel1.txt',word_count)
    download.start_download('http://0.0.0.0:8000/novel2.txt',word_count)
    download.start_download('http://0.0.0.0:8000/novel3.txt',word_count)

    # 使用 python3 -m http.server 启动当前文件夹 作为 网址，可以进行下载或查看