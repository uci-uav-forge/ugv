import tkinter
from turtle import hideturtle
from unicodedata import name
import tkintermapview

root = tkinter.Tk()
root.title('UGV COORDINATES')
root.geometry("900x700")

my_label = tkinter.LabelFrame(root)
my_label.pack(pady=20)

map_widget = tkintermapview.TkinterMapView(my_label, width=800, height=600, corner_radius=0)
map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)  # google satellite

# map_widget.set_position(33.64592419847688, -117.84267821814397) 
map_widget.set_position(38.145103, -76.427856) 
map_widget.set_zoom(16)
map_widget.set_polygon([(38.14616666666666, -76.42666666666668), (38.14636111111111, -76.42616666666667), (38.14558333333334, -76.42608333333334), (38.14541666666667, -76.42661111111111)], name="UGV/AirDrop Boundary")
map_widget.set_marker(38.146152, -76.426396)


map_widget.pack()



root.mainloop()
