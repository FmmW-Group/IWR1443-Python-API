# IWR1443-Python-API
一个使用Python读取IWR1443毫米波雷达串口数据的代码库，不需要安装任何的sdk（实际上，当你购买到IWR1443的时候，芯片会内置相关代码）。
代码修改自[仓库](https://github.com/ibaiGorordo/IWR1443-Read-Data-Python-MMWAVE-SDK-1)，目前代码适用于使用sdk1.xx以及sdk2.xx的所有IWR1443boost版本。

# Quick Start
安装pyserial
```pip install pyserial```
运行代码：
```python ReadIWRSDK2.py```


# main函数解析
```
def main():
    configFileName = './radarconfig/1443config.cfg'
    IWR1443 = ReadIWR14xx(configFileName,CLIport="COM5", Dataport="COM6")
    sleeptime = 0.001*IWR1443.framePeriodicity
    dataOk, frameNumber, detObj = IWR1443.read()
    while True:
        try:
            dataOk, frameNumber, detObj = IWR1443.read()
            if dataOk:
                data = np.array([detObj['x'], detObj['y'], detObj['z'], detObj['doppler'], detObj['peakVal']]).transpose(1, 0)
                print(frameNumber, np.shape(data))
            else:
                print(0)
                
            time.sleep(sleeptime) # Sampling frequency of 20 Hz
        except KeyboardInterrupt:
            del IWR1443
            break
```
