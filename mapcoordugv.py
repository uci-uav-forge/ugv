import tkinter
from turtle import hideturtle
import tkintermapview

root = tkinter.Tk()
root.title('UGV COORDINATES')
root.geometry("900x700")

my_label = tkinter.LabelFrame(root)
my_label.pack(pady=20)

map_widget = tkintermapview.TkinterMapView(my_label, width=800, height=600, corner_radius=0)
map_widget.set_position(33.64592419847688, -117.84267821814397)
map_widget.set_zoom(16)

map_widget.pack()



root.mainloop()
