#! /usr/bin/env python
# -*- coding: utf-8 -*-
import time
import os

import cv2
import numpy as np
import win32api
import win32con
from vkcode import VK_CODE


class Blhx:
    def __init__(self, adbPATH, sharedPATH):
        self.adb = adbPATH
        self.sharedPATH = sharedPATH

    def getScreen(self, filePath="/sdcard/Pictures/tmp.png"):
        os.system(f"{self.adb} -e shell screencap -p {filePath}")
        time.sleep(0.5)
        return cv2.imread(f"{self.sharedPATH}\\tmp.png")

    def tap(self, X, Y):
        os.system(f"{self.adb} -e shell input tap {X} {Y}")

    def clipEvery(self, fullImg):
        startX = 123 + 20
        startY = 189 + 20
        w = 96 - 70
        h = 97 - 70
        intervalX = 117
        intervalY = 118
        matchImgs = []
        matchedImgs = []
        mark = 1
        for y in range(4):
            for x in range(9):

                index = y * 9 + x
                sY = startY + intervalY * y
                sX = startX + intervalX * x
                eY = sY + w
                eX = sX + h
                img = fullImg[sY:eY, sX:eX, :]
                grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                _, img2 = cv2.threshold(grayImg, 127, 255, cv2.THRESH_BINARY)

                thisImgInfo = [y, x, img2]
                for ii, imInfo in enumerate(matchImgs):
                    im = imInfo[2]
                    # print(im)
                    if im.any():
                        diff = cv2.subtract(im, img2)
                        result = not np.any(diff)
                        # result表示两个图片是否相等
                        if result:
                            matchedImgs.append(thisImgInfo[:2])
                            matchedImgs.append(imInfo[:2])
                            del matchImgs[ii]
                            break
                else:
                    matchImgs.append(thisImgInfo)
        if matchedImgs.__len__() == 34:
            matchedImgs.append(matchImgs[0][:2])
            matchedImgs.append(matchImgs[1][:2])
        if not matchedImgs.__len__() == 36:
            print("没匹配够36个")
        return matchedImgs

    def parseLLK(self, matchedImgs):
        parsed = np.zeros((6, 11))
        # for i, imInfo in enumerate(matchedImgs):
        #     parsed[imInfo[0] + 1, imInfo[1] + 1] = int(i // 2) + 1
        for i in range(matchedImgs.__len__()):
            imInfo = matchedImgs[i]
            parsed[imInfo[0] + 1, imInfo[1] + 1] = int(i // 2) + 1
            matchedImgs[i][0] += 1
            matchedImgs[i][1] += 1
        return parsed, matchedImgs


    def findLineFetch(self, start, arr, dire=0):
        """获取某个点在水平或竖直方向上能得到的所有点(无拐点的那种)"""
        # 如果dire是0,则在横竖方向上都搜索.
        # 如果dire是1,则只在竖直方向上搜索
        # 如果dire是2,则只在水平方向上搜索
        y = start[0]
        x = start[1]

        # 搜索到的不为0,将其值放入该数组
        # 水平方向上的
        vs0 = []
        # 竖直方向上的
        vs1 = []

        # 在水平方向上搜索到的0,将其坐标放入该数组
        l0s = []

        # 在竖直方向上搜索到的0,将其坐标放入该数组
        l1s = []

        # 在水平方向上搜索
        if dire == 0 or dire == 2:

            # 水平 向左搜索
            l = x
            while l > 0:
                # print(arr)
                v = arr[y, l - 1]
                if v == 0:
                    l0s.append([y, l - 1])
                    l -= 1
                else:
                    vs0.append(v)
                    break

            # 水平 向右搜索
            r = x
            while r < 10:
                v = arr[y, r + 1]
                if v == 0:
                    l0s.append([y, r + 1])
                    r += 1
                else:
                    vs0.append(v)
                    break

        # 在竖直方向上搜索
        if dire == 0 or dire == 1:

            # 竖直 向上搜索
            u = y
            while u > 0:
                v = arr[u - 1, x]
                if v == 0:
                    l1s.append([u - 1, x])
                    u -= 1
                else:
                    vs1.append(v)
                    break

            # 竖直 向下搜索
            d = y
            while d < 5:
                v = arr[d + 1, x]
                if v == 0:
                    l1s.append([d + 1, x])
                    d += 1
                else:
                    vs1.append(v)
                    break

        # return 水平方向搜索来的0的坐标们的数组 , 竖直方向上搜索来的0的坐标们的数组 , 水平方向上接触到的不为0的数字 , 竖直方向上接触到的不为0的数字
        return l0s, l1s, vs0, vs1

    def pointSearch(self, pos, parsed):
        """获取某个点能到达的所有点,有两次拐点"""

        # 水平方向搜索来的0的坐标们的数组 , 竖直方向上搜索来的0的坐标们的数组 , 水平方向上接触到的不为0的数字 , 竖直方向上接触到的不为0的数字
        l0s, l1s, vs0, vs1 = self.findLineFetch(pos, parsed, dire=0)
        vs = vs0 + vs1

        # 一开始往水平方向走 -> 第一个拐点处
        l2s = []
        for p in l0s:
            # 对每一个 l0s 中的点做竖直方向上的查找
            _, l1sM, _, vsM = self.findLineFetch(p, parsed, dire=1)
            l2s += l1sM
            vs += vsM

        for p in l2s:
            # 对每一个 l2s 中的点做水平方向上的查找
            _, _, vs2, _ = self.findLineFetch(p, parsed, dire=2)
            vs += vs2

        # 一开始往竖直方向走 -> 第一个拐点处
        l3s = []
        for p in l1s:
            # 对每一个 l1s 中的点做水平方向上的查找
            l3sM, _, vsM, _ = self.findLineFetch(p, parsed, dire=2)
            l3s += l3sM
            vs += vsM
        for p in l3s:
            # 对每一个 l3s 中的点做竖直方向上的查找
            _, _, _, vs2 = self.findLineFetch(p, parsed, dire=1)
            vs += vs2
        return vs

    def findAll(self, parsed, matched):
        res = []
        while matched.__len__():
            l = matched.__len__()
            for i in range(int(matched.__len__() // 2)):
                start = matched[i * 2]
                target = matched[i * 2 + 1]
                value = parsed[start[0], start[1]]
                vs = self.pointSearch(start, parsed)
                if value in vs:
                    res.append(start)
                    res.append(target)
                    parsed[start[0], start[1]], parsed[target[0], target[1]] = 0, 0
                    del matched[i * 2 + 1]
                    del matched[i * 2]
                    break
            else:
                return np.any(parsed),res

        return np.any(parsed),res

    def run(b):
        b.tap(1160, 529)
        time.sleep(4.2)
        print("print screen")
        i = b.getScreen()
        matchedImgs = b.clipEvery(i)
        parsed, matched = b.parseLLK(matchedImgs)
        print(parsed, matched)
        tf, res = b.findAll(parsed, matched)
        if tf:
            print("bad game")
            b.tap(32,24)
            time.sleep(5)
            return b.run()
        st = 0.26
        for i in range(int(res.__len__() // 2)):
            s = res[i * 2]
            t = res[i * 2 + 1]

            kNode0 = keyMap[s[0] - 1][s[1] - 1]
            kNode1 = keyMap[t[0] - 1][t[1] - 1]

            win32api.keybd_event(VK_CODE[kNode0], 0, 0, 0)
            time.sleep(0.04)
            win32api.keybd_event(VK_CODE[kNode1], 0, 0, 0)
            time.sleep(0.04)
            win32api.keybd_event(VK_CODE[kNode0], 0, win32con.KEYEVENTF_KEYUP, 0)
            time.sleep(0.04)
            win32api.keybd_event(VK_CODE[kNode1], 0, win32con.KEYEVENTF_KEYUP, 0)
            time.sleep(st)

keyMap = [
    "qwertyuio",
    "asdfghjkl",
    "zxcvbnm,.",
    "123456789"
]

if __name__ == '__main__':
    b = Blhx(r"E:\programfiles\adb\adb.exe", "J:\ldmnqshare")
    b.run()
