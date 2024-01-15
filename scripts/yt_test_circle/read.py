file = open('/home/ylc/Reinforcement/simulator/controllers/yt_lebai/vw.txt')
datavw = []
for line in file.readlines():
    print(line)
    curline = line.strip().split(" ")
    datavw.append(curline[:])
file.close()
print(len(datavw))