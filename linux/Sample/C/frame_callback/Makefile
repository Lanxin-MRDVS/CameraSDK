app = frame_callback
path = /opt/MRDVS

Demo: $(app).cpp
	g++ -std=c++11 -g -o demo_$(app) $(app).cpp -I $(path)/include -L $(path)/lib -lLxCameraApi -lpthread

clean:
	rm demo_$(app) -rf
