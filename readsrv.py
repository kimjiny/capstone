import pandas as pd 
from datetime import datetime
import re
srt_file = "/home/park/문서/DCIM (1)/DJI_202501241632_009/DJI_20250124163426_0004_V.SRT"\

with open(srt_file,"r",encoding="utf-8") as file:
    lines = file.readlines()
data = []
for i in range(0,len(lines),6):
    if i+5 <len(lines):
        # print(lines[i+1])
        #인덱스 번호
        index = lines[i].strip()
        # time 테이블 표시
        timetable = lines[i+1].strip()
        timetable = timetable.split("-->")[0] 
        timetable = timetable.replace(",",".").split(":")
        hh = int(timetable[0])
        mm = int(timetable[1])
        ss = float(timetable[2])
        
        timetable = hh*60*60+mm*60+ss
        
        
        # print(timetable)
        # 연월일
        date = lines[i+3].strip()
        dt = datetime.strptime(date,"%Y-%m-%d %H:%M:%S.%f")
        year = dt.year
        month = dt.month
        day = dt.day
        hour = dt.hour
        minute = dt.minute
        second = dt.second
        microsecond = dt.microsecond  # 소수점 이하 마이크로초
        # print(f"연: {year}, 월: {month}, 일: {day}, 시: {hour}, 분: {minute}, 초: {second}, 마이크로초: {microsecond}")
        maps = lines[i+4]
        #위도 경도
        # longitude와 latitude를 한 번에 매칭하는 정규식
        pattern = r"\[latitude:\s*([\d]+\.\d+)\]\s*\[longitude:\s*([\d]+\.\d+)\]"
        

        latitude,longitude = re.search(pattern, maps).groups()
        data.append([timetable,latitude,longitude])
        
df = pd.DataFrame(data)
print(df)

            
        
        
