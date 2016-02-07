import wx
import socket
import threading
from networktables import NetworkTable
import webbrowser
import sys as LOL
class main_window(wx.Frame):
    def __init__(self,parent,id):
        wx.Frame.__init__(self,parent,id,"Robot",size=(250,200))
        self.frame=wx.Panel(self)
        self.front_left=wx.TextCtrl(self.frame,-1,pos=(100,25),size=(140,25),style=wx.NO_BORDER|wx.TE_READONLY)
        distance_words=wx.StaticText(self.frame,-1,"Distance",pos=(10,25))
        self.front_left.SetBackgroundColour("green")


        self.camera_stuff=wx.TextCtrl(self.frame,-1,pos=(100,75),size=(140,25),style=wx.NO_BORDER|wx.TE_READONLY)
        vision_words=wx.StaticText(self.frame,-1,"Vision",pos=(10,75))
        self.camera_stuff.SetBackgroundColour("green")

        self.user_font=wx.Font(18, wx.DEFAULT ,wx.NORMAL, wx.NORMAL)
        distance_words.SetFont(self.user_font)
        self.front_left.SetFont(self.user_font)
        self.camera_stuff.SetFont(self.user_font)
        vision_words.SetFont(self.user_font)

        self.camera=wx.Button(self.frame,label="Camera",pos=(50,120),size=(150,50))
        self.Bind(wx.EVT_BUTTON, self.camera_link,self.camera)


        self.Bind(wx.EVT_CLOSE, self.close_window)
        #Pynetwork tables stuff
        NetworkTable.setIPAddress("roborio-4480-frc.lan")
        NetworkTable.setClientMode()
        NetworkTable.initialize()

        self.smrt = NetworkTable.getTable("SmartDashboard")
        self.distance = self.smrt.getAutoUpdateValue('Distance', 0)
        self.vision = self.smrt.getAutoUpdateValue('Vision', 0)
        threader=threading.Thread(target=self.threading)
        self.keep_open=True
        threader.start()
    def threading(self):
        while self.keep_open:
            dist = self.distance.value
            camera_vision = self.vision.value
            if dist < 10:
                self.front_left.SetBackgroundColour("green")
            else:
                self.front_left.SetBackgroundColour("red")
            self.front_left.SetValue(str(dist))
            if camera_vision > 220:
                self.camera_stuff.SetValue("To the Left")
                self.camera_stuff.SetBackgroundColour("red")
            elif camera_vision < 140 and camera_vision != 0:
                self.camera_stuff.SetValue("To the Right")
                self.camera_stuff.SetBackgroundColour("red")
            elif camera_vision == 0:
                self.camera_stuff.SetValue("Goal not found")
                self.camera_stuff.SetBackgroundColour("yellow")
            else:
                self.camera_stuff.SetValue("FIRE FIRE")
                self.camera_stuff.SetBackgroundColour("green")

        self.Refresh()
    def camera_link(self,event):
        webbrowser.open_new_tab("roborio-4480-frc.local:5800")

    def close_window(self, event):
        self.keep_open=False
        LOL.exit()

if __name__=="__main__":
    app=wx.App(False)
    window=main_window(parent=None, id=-1)
    window.Show()
    app.MainLoop()  
