    def autonomousPeriodic(self):
        self.vision()
        if self.state==0:
            self.timer.reset()
            self.state=1
        elif self.state==1:
            self.auto1=-.5
            self.auto2=-.5
            if self.timer.hasPeriodPassed(2):
                self.state=2
        elif self.state==2:
            if self.alignY():
                self.state=3
        elif self.state==3:
            if self.alignX():
                self.state=4
    
    
    def vision(self):

        try:
            self.vision_table.retrieveValue('centerX', self.vision_x)
            self.vision_table.retrieveValue('centerY', self.vision_y)
        except KeyError:
            pass
    def alignX(self):
        if len(self.vision_x)>0:
            if len(self.vision_x)==1:
                self.vision_numberX=self.vision_x[0]
            else:
                good=self.vision_x[0]
                normal=abs(self.vision_x[0]-160)
                for i in self.vision_x[1:]:
                    total=abs(i-160)
                    if total <= normal:
                        normal=total
                        good=i
                self.vision_numberX=good
            if self.vision_numberX > 180:
                self.auto_calc=(((self.vision_numberX-180)/140)*.25)+.25
                self.auto1=self.auto_calc
                self.auto2=(self.auto_calc)

            elif self.vision_numberX < 150:
                self.auto_calc=(((150-self.vision_numberX)/150)*.25)+.25
                self.auto1=(-1*self.auto_calc)
                self.auto2=(-1*self.auto_calc)
            else:
                return True
    def alignY(self):
        if len(self.vision_x)>0:
            self.vision_numberY=self.vision_y[0]

            if self.vision_numberY > 230:
                self.auto_calc=(((self.vision_numberY-230)/90)*.25)+.25
                self.auto1=(-1*self.auto_calc)
                self.auto2=(self.auto_calc)

            elif self.vision_numberY < 200:
                self.auto_calc=(((200-self.vision_numberY)/200)*.25)+.25
                self.auto1=(self.auto_calc)
                self.auto2=(-1*self.auto_calc)
            else:
                return True
