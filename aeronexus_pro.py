import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import math, random, time, csv, threading, webbrowser
from collections import deque
from datetime import datetime
try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None
try:
    import tkintermapview
except ImportError:
    tkintermapview = None
try:
    import cv2
except ImportError:
    cv2 = None
try:
    from PIL import Image, ImageTk
except ImportError:
    Image = ImageTk = None
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

BG='#070c16'; PANEL='#101a2a'; PANEL2='#15243a'; EDGE='#273953'; TEXT='#e8f1ff'; MUTED='#8296b3'; CYAN='#36d9ff'; GREEN='#55efb0'; AMBER='#ffc66d'; RED='#ff617c'; PURPLE='#a68bff'

class AERONIQ:
    def __init__(self, root):
        self.root=root; root.title('AERONIQ'); root.geometry('1500x940'); root.minsize(1180,760); root.configure(bg=BG)
        self.alive=True; self.demo=True; self.connected=False; self.link=None; self.log=None; self.writer=None; self.t=0; self.last_alert=''; self.alerts=[]
        self.d={'alt':18.2,'speed':5.4,'battery':86.0,'voltage':15.4,'heading':142.0,'roll':2.5,'pitch':-1.0,'lat':-7.2756,'lon':112.7970,'sats':14,'mode':'GUIDED','armed':False,'climb':0.2,'fix':'3D FIX','rssi':78.0,'rssi_raw':198,'gx':0.0,'gy':0.0,'gz':0.0}
        # Demo traces use irregular, non-periodic motion with occasional trend changes.
        # No sine/cosine cycles: a bounded random walk is more like a noisy telemetry trace.
        self.sim={'alt':18.2,'speed':5.4,'gx':0.0,'gy':0.0,'gz':0.0,
                  'roll':2.5,'pitch':-1.0,'rssi':78.0,
                  'alt_drift':0.0,'speed_drift':0.0,'gx_drift':0.0,'gy_drift':0.0,'gz_drift':0.0}
        def seed_trace(center, spread, low=0.0, high=None):
            values=[]
            current=center
            drift=0.0
            for i in range(240):
                if i % random.randint(8, 22) == 0:
                    drift=random.uniform(-spread*0.18, spread*0.18)
                current += drift + random.gauss(0, spread*0.30)
                # Softly reflect at bounds rather than pulling every sample back to center.
                if current < low:
                    current=low + (low-current)*0.25
                    drift=abs(drift)*0.35
                if high is not None and current > high:
                    current=high - (current-high)*0.25
                    drift=-abs(drift)*0.35
                current=max(low,current)
                if high is not None: current=min(high,current)
                values.append(current)
            return deque(values,maxlen=240)
        self.hist={'alt':seed_trace(18.2,1.25,0,60),
                   'speed':seed_trace(5.4,0.85,0,30),
                   'battery':deque([86.0]*240,maxlen=240)}
        self.gyro_hist={k:seed_trace(0.0,5.0,-30,30) for k in ('gx','gy','gz')}
        # Start the live simulation at the right edge of the prefilled history.
        # This avoids a visible vertical jump when the first live samples arrive.
        for axis in ('gx','gy','gz'):
            initial=float(self.gyro_hist[axis][-1])
            self.d[axis]=initial
            self.sim[axis]=initial
        for key in ('alt','speed'):
            self.d[key]=float(self.hist[key][-1])
            self.sim[key]=float(self.hist[key][-1])
        self.track=deque(maxlen=100); self.track.append((self.d['lat'],self.d['lon']))
        self.map_marker_over=None; self.map_path_over=None; self.map_marker_mission=None; self.map_path_mission=None; self.last_map_refresh=0.0
        self.camera_capture=None; self.camera_thread=None; self.camera_running=False; self.camera_frame=None
        self.location_history=[]
        self.camera_lock=threading.Lock(); self.camera_photo=None; self.camera_after=None
        self.face_detection_enabled=False; self.face_count=0; self.face_cascade=None
        self.drone_count=0; self.drone_model=None; self.drone_model_status='MODEL NOT LOADED'
        self._style(); self._build(); self._tick(); root.protocol('WM_DELETE_WINDOW',self.close)
    def _style(self):
        s=ttk.Style(); s.theme_use('clam'); s.configure('TNotebook',background=BG,borderwidth=0); s.configure('TNotebook.Tab',background=PANEL2,foreground=MUTED,padding=(18,10),font=('Segoe UI',9,'bold')); s.map('TNotebook.Tab',background=[('selected',CYAN)],foreground=[('selected','#06111c')]); s.configure('TCombobox',fieldbackground=PANEL2,background=PANEL2,foreground=TEXT,arrowcolor=CYAN)
    def panel(self,parent,title,tag='LIVE'):
        f=tk.Frame(parent,bg=PANEL,highlightbackground=EDGE,highlightthickness=1); h=tk.Frame(f,bg=PANEL); h.pack(fill='x',padx=13,pady=(11,7)); tk.Label(h,text=title.upper(),bg=PANEL,fg=TEXT,font=('Segoe UI',10,'bold')).pack(side='left'); tk.Label(h,text=tag,bg=PANEL,fg=CYAN,font=('Consolas',8,'bold')).pack(side='right'); tk.Frame(f,bg=CYAN,height=2).pack(fill='x',padx=13); return f
    def btn(self,p,text,cmd,bg=PANEL2,fg=TEXT): return tk.Button(p,text=text,command=cmd,bg=bg,fg=fg,activebackground=EDGE,activeforeground=TEXT,relief='flat',bd=0,font=('Segoe UI',9,'bold'),cursor='hand2',pady=9)
    def _build(self):
        header=tk.Frame(self.root,bg='#0b1422',height=76); header.pack(fill='x'); header.pack_propagate(False)
        tk.Label(header,text='✦',bg='#0b1422',fg=CYAN,font=('Segoe UI Symbol',30,'bold')).pack(side='left',padx=(20,12))
        brand=tk.Frame(header,bg='#0b1422'); brand.pack(side='left',pady=12); tk.Label(brand,text='AERONIQ',bg='#0b1422',fg=TEXT,font=('Segoe UI',18,'bold')).pack(anchor='w'); tk.Label(brand,text='Aero Neural Intelligence Quotient',bg='#0b1422',fg=MUTED,font=('Consolas',8)).pack(anchor='w')
        self.clock=tk.Label(header,text='SYSTEM TIME --:--:--',bg='#0b1422',fg=MUTED,font=('Consolas',10)); self.clock.pack(side='right',padx=20)
        self.badge=tk.Label(header,text='● DEMO SIMULATION',bg='#12302e',fg=GREEN,font=('Consolas',10,'bold'),padx=13,pady=9); self.badge.pack(side='right',padx=12)
        # Compact battery gauge + signal bars inspired by the user's reference.
        self.mini_telemetry=tk.Canvas(self.root,width=280,height=74,bg=BG,highlightthickness=0)
        self.mini_telemetry.pack(anchor='e',padx=18,pady=(4,0))
        # status ribbon
        ribbon=tk.Frame(self.root,bg=BG); ribbon.pack(fill='x',padx=16,pady=12); self.status={}
        for key,label in [('flight','FLIGHT STATE'),('mode','FLIGHT MODE'),('gps','GPS QUALITY'),('link','DATA LINK')]:
            box=tk.Frame(ribbon,bg=PANEL,highlightbackground=EDGE,highlightthickness=1); box.pack(side='left',fill='x',expand=True,padx=(0,7)); tk.Label(box,text=label,bg=PANEL,fg=MUTED,font=('Consolas',8,'bold')).pack(anchor='w',padx=12,pady=(8,2)); v=tk.Label(box,text='STANDBY',bg=PANEL,fg=GREEN,font=('Segoe UI',12,'bold')); v.pack(anchor='w',padx=12,pady=(0,8)); self.status[key]=v
        tabs=ttk.Notebook(self.root); tabs.pack(fill='both',expand=True,padx=16,pady=(0,10)); self.tab_over=tk.Frame(tabs,bg=BG); self.tab_mission=tk.Frame(tabs,bg=BG); self.tab_system=tk.Frame(tabs,bg=BG); self.tab_imu=tk.Frame(tabs,bg=BG); tabs.add(self.tab_over,text='  FLIGHT OVERVIEW  '); tabs.add(self.tab_system,text='  SYSTEM & LOGS  ')
        self._overview(); self._mission(); self._imu(); self._system()
        foot=tk.Frame(self.root,bg='#0b1422',height=28); foot.pack(fill='x',side='bottom'); foot.pack_propagate(False); tk.Label(foot,text='Build by Beryl Dumanao',bg='#0b1422',fg=MUTED,font=('Consolas',8)).pack(side='left',padx=16,pady=6); self.foot=tk.Label(foot,text='SYSTEM NOMINAL',bg='#0b1422',fg=GREEN,font=('Consolas',8,'bold')); self.foot.pack(side='right',padx=16)
    def metric(self,p,title,unit,color):
        f=tk.Frame(p,bg=PANEL2,highlightbackground=EDGE,highlightthickness=1); f.pack(side='left',fill='both',expand=True,padx=4,pady=4); tk.Label(f,text=title,bg=PANEL2,fg=MUTED,font=('Consolas',8,'bold')).pack(anchor='w',padx=12,pady=(10,1)); val=tk.Label(f,text='--',bg=PANEL2,fg=color,font=('Segoe UI',24,'bold')); val.pack(anchor='w',padx=12); tk.Label(f,text=unit,bg=PANEL2,fg=MUTED,font=('Consolas',8)).pack(anchor='w',padx=12,pady=(0,9)); return val
    def _overview(self):
        # Make the entire Flight Overview page scrollable, not only the charts.
        overview_shell=tk.Frame(self.tab_over,bg=BG)
        overview_shell.pack(fill='both',expand=True)
        overview_canvas=tk.Canvas(overview_shell,bg=BG,highlightthickness=0)
        overview_scroll=ttk.Scrollbar(overview_shell,orient='vertical',command=overview_canvas.yview)
        overview_canvas.configure(yscrollcommand=overview_scroll.set)
        overview_scroll.pack(side='right',fill='y')
        overview_canvas.pack(side='left',fill='both',expand=True)
        overview_content=tk.Frame(overview_canvas,bg=BG)
        overview_window=overview_canvas.create_window((0,0),window=overview_content,anchor='nw')
        def _overview_content_configure(_event=None):
            overview_canvas.configure(scrollregion=overview_canvas.bbox('all'))
        def _overview_canvas_width(event):
            overview_canvas.itemconfigure(overview_window,width=event.width)
        overview_content.bind('<Configure>',_overview_content_configure)
        overview_canvas.bind('<Configure>',_overview_canvas_width)
        def _overview_wheel(event):
            try:
                if overview_canvas.winfo_exists():
                    overview_canvas.yview_scroll(int(-1*(event.delta/120)), 'units')
            except tk.TclError:
                pass
        overview_canvas.bind_all('<MouseWheel>',_overview_wheel)
        self.overview_canvas=overview_canvas

        metrics=tk.Frame(overview_content,bg=BG); metrics.pack(fill='x',pady=(7,8)); self.big={}
        for k,title,unit,color in [('alt','ALTITUDE','METERS',CYAN),('speed','GROUND SPEED','M/S',GREEN)]: self.big[k]=self.metric(metrics,title,unit,color)

        row=tk.Frame(overview_content,bg=BG); row.pack(fill='x',expand=False)
        # Main workspace occupies the left and center; quick actions remain on the right.
        workspace=tk.Frame(row,bg=BG); workspace.pack(side='left',fill='both',expand=True,padx=(0,8))
        right=tk.Frame(row,bg=BG,width=280); right.pack(side='left',fill='y'); right.pack_propagate(False)

        top=tk.Frame(workspace,bg=BG)
        top.pack(fill='x',expand=False,pady=(0,8))
        # Left column (3D Drone + Drone Location) is wider than Live Camera.
        top.grid_columnconfigure(0, weight=6, uniform='overview_columns')
        top.grid_columnconfigure(1, weight=4, uniform='overview_columns')
        top.grid_rowconfigure(0, weight=1)
        left_stack=tk.Frame(top,bg=BG)
        left_stack.grid(row=0,column=0,sticky='nsew',padx=(0,8))
        drone_panel=self.panel(left_stack,'3D Drone / Rotor Monitor','ANIMATED QUADCOPTER')
        drone_panel.pack(fill='x',expand=False,pady=(0,8))
        drone_panel.configure(height=250); drone_panel.pack_propagate(False)
        self.hud=tk.Canvas(drone_panel,bg='#07111d',height=215,highlightthickness=0)
        self.hud.pack(fill='both',expand=True,padx=8,pady=8); self.hud.bind('<Configure>',lambda e:self.draw_hud())

        location_panel=self.panel(left_stack,'Drone Location','FLIGHT TRACK')
        location_panel.pack(fill='x',expand=False)
        location_panel.configure(height=250); location_panel.pack_propagate(False)
        location_body=tk.Frame(location_panel,bg='#050b13')
        location_body.pack(fill='both',expand=True,padx=8,pady=8)
        self.location_canvas=tk.Canvas(location_body,bg='#050b13',highlightthickness=0,height=180)
        self.location_canvas.pack(fill='both',expand=True)
        self.location_canvas.bind('<Configure>',lambda e:self.draw_location_track())
        # GPS SIMULATION badge and OPEN MAP button intentionally omitted.

        camera_panel=self.panel(top,'Live Camera','CAMERA FEED')
        camera_panel.grid(row=0,column=1,sticky='nsew')
        camera_panel.configure(height=508); camera_panel.pack_propagate(False)
        camera_body=tk.Frame(camera_panel,bg='#050b13')
        camera_body.pack(fill='both',expand=True,padx=8,pady=8)
        self.camera_view=tk.Label(
            camera_body,
            text='LIVE CAMERA OFFLINE\n\nSelect camera source and press START CAMERA',
            bg='#050b13', fg=CYAN, font=('Consolas',11,'bold'),
            justify='center', compound='center'
        )
        self.camera_view.pack(fill='both',expand=True)
        camera_controls=tk.Frame(camera_panel,bg=PANEL)
        camera_controls.pack(fill='x',padx=8,pady=(0,8))
        tk.Label(camera_controls,text='CAMERA SOURCE',bg=PANEL,fg=MUTED,font=('Consolas',7,'bold')).pack(anchor='w')
        control_row=tk.Frame(camera_controls,bg=PANEL)
        control_row.pack(fill='x',pady=(4,2))
        self.camera_source_type=tk.StringVar(value='USB WEBCAM')
        source_picker=ttk.Combobox(control_row,textvariable=self.camera_source_type,values=['USB WEBCAM','DRONE / RTSP'],state='readonly',width=15)
        source_picker.pack(side='left',padx=(0,6),ipady=3)
        source_picker.bind('<<ComboboxSelected>>',self._camera_source_changed)
        self.camera_source=tk.StringVar(value='0')
        self.camera_source_entry=tk.Entry(control_row,textvariable=self.camera_source,bg=PANEL2,fg=TEXT,insertbackground=CYAN,relief='flat')
        self.camera_source_entry.pack(side='left',fill='x',expand=True,ipady=7,padx=(0,6))
        self.btn(control_row,'START CAMERA',self.start_camera,GREEN,'#06111c').pack(side='left',padx=(0,5))
        self.btn(control_row,'STOP',self.stop_camera).pack(side='left')
        face_row=tk.Frame(camera_controls,bg=PANEL)
        face_row.pack(fill='x',pady=(3,0))
        self.face_count_label=tk.Label(face_row,text='PERSONS: --  |  DRONES: --',bg=PANEL,fg=MUTED,font=('Consolas',8,'bold'))
        self.face_count_label.pack(side='right')
        self.camera_status=tk.Label(camera_controls,text='CAMERA STATUS: OFFLINE • SELECT USB OR DRONE RTSP',bg=PANEL,fg=MUTED,font=('Consolas',7))
        self.camera_status.pack(anchor='w',pady=(3,0))

        # Four equal telemetry panels occupy the lower dashboard area.
        # The entire overview page scrolls vertically, while each chart stays fixed.
        charts_shell=tk.Frame(workspace,bg=BG)
        charts_shell.pack(fill='x',expand=False,pady=(0,2))
        charts_shell.grid_columnconfigure(0,weight=1,uniform='telemetry_cols')
        charts_shell.grid_columnconfigure(1,weight=1,uniform='telemetry_cols')
        charts_shell.grid_rowconfigure(0,weight=1)
        charts_shell.grid_rowconfigure(1,weight=1)
        self.charts={}
        chart_specs=[
            ('alt','ALTITUDE INDICATORS','60 S  /  RAW · EMA 5s · EMA 15s',0,0),
            ('speed','SPEED INDICATORS','60 S  /  RAW · EMA 5s · EMA 15s',0,1),
        ]
        for k,title,tag,r,col in chart_specs:
            p=self.panel(charts_shell,title,tag)
            p.grid(row=r,column=col,sticky='nsew',padx=(0,6) if col==0 else (0,0),pady=(0,8))
            c=tk.Canvas(p,bg=PANEL,height=145,highlightthickness=0)
            c.pack(fill='both',expand=True,padx=6,pady=(0,6))
            self.charts[k]=(c,CYAN if k=='alt' else GREEN)
            c.bind('<Configure>',lambda e,key=k:self.draw_chart(key))

        # IMU / Gyroscope now spans the full chart row; Battery / Signal Health removed.
        imu_panel=self.panel(charts_shell,'IMU / GYROSCOPE','X RED  /  Y GREEN  /  Z BLUE')
        imu_panel.grid(row=1,column=0,columnspan=2,sticky='nsew',pady=(0,8))
        self.imu_mini_canvas=tk.Canvas(imu_panel,bg='#07111d',height=220,highlightthickness=0)
        self.imu_mini_canvas.pack(fill='both',expand=True,padx=6,pady=(0,6))
        self.imu_mini_canvas.bind('<Configure>',lambda e:self.draw_imu(self.imu_mini_canvas))

        p=self.panel(right,'Link & quick actions','READ-ONLY'); p.pack(fill='x',pady=(0,8))
        c=tk.Frame(p,bg=PANEL); c.pack(fill='x',padx=12,pady=(6,13))
        self.transport=tk.StringVar(value='UDP'); self.endpoint=tk.StringVar(value='127.0.0.1:14550')
        tk.Label(c,text='TRANSPORT',bg=PANEL,fg=MUTED,font=('Consolas',8)).pack(anchor='w')
        ttk.Combobox(c,textvariable=self.transport,values=['UDP','TCP','Serial'],state='readonly').pack(fill='x',pady=(3,8))
        tk.Label(c,text='ENDPOINT / PORT',bg=PANEL,fg=MUTED,font=('Consolas',8)).pack(anchor='w')
        tk.Entry(c,textvariable=self.endpoint,bg=PANEL2,fg=TEXT,insertbackground=CYAN,relief='flat').pack(fill='x',ipady=8,pady=(3,8))
        self.btn(c,'CONNECT MAVLINK',self.connect,CYAN,'#06111c').pack(fill='x')
        self.btn(c,'SWITCH TO DEMO',self.demo_mode).pack(fill='x',pady=(6,0))
        tk.Label(c,text='Telemetry only • no flight commands',bg=PANEL,fg=AMBER,font=('Segoe UI',8)).pack(anchor='w',pady=(9,0))
        p=self.panel(right,'Live alerts','HEALTH MONITOR'); p.pack(fill='both',expand=True)
        self.alertbox=tk.Text(p,bg=PANEL,fg=TEXT,relief='flat',height=10,font=('Consolas',9),wrap='word',state='disabled')
        self.alertbox.pack(fill='both',expand=True,padx=10,pady=10)

    def _camera_source_changed(self, _event=None):
        if self.camera_source_type.get() == 'USB WEBCAM':
            self.camera_source.set('0')
            self.camera_source_entry.config(state='normal')
        else:
            self.camera_source.set('rtsp://username:password@IP:554/stream')
            self.camera_source_entry.config(state='normal')

    def start_camera(self):
        if cv2 is None or Image is None or ImageTk is None:
            messagebox.showerror(
                'Camera dependencies missing',
                'Live camera and face detection require OpenCV and Pillow. Run RUN_AERONIQ.bat again to install requirements.'
            )
            return
        self.stop_camera(update_status=False)
        source_text=self.camera_source.get().strip() or '0'
        if self.camera_source_type.get() == 'USB WEBCAM':
            try:
                source=int(source_text) if source_text.isdigit() else 0
            except ValueError:
                source=0
        else:
            source=source_text
            if not source.lower().startswith(('rtsp://','http://','https://','rtmp://')):
                messagebox.showerror('Invalid drone stream','Masukkan URL RTSP/HTTP stream kamera drone, contohnya rtsp://192.168.1.10:554/stream')
                return
        try:
            cap=cv2.VideoCapture(source)
            if not cap.isOpened():
                cap.release()
                raise RuntimeError('Kamera tidak dapat dibuka. Periksa izin webcam, indeks USB, alamat IP, jaringan, dan URL stream.')
            self.camera_capture=cap
            self.camera_running=True
            self.face_count=0
            # Face detection is automatically enabled whenever a camera is started.
            cascade_path=cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            self.face_cascade=cv2.CascadeClassifier(cascade_path)
            if self.face_cascade.empty():
                self.face_detection_enabled=False
                raise RuntimeError('Model deteksi wajah OpenCV tidak ditemukan.')
            self.face_detection_enabled=True
            self.face_count=0
            self.drone_count=0
            self.drone_model=None
            self.drone_model_status='MODEL NOT FOUND'
            # Drone detection uses an optional custom YOLO model. Put drone.pt or best.pt
            # next to this script; a generic COCO model does not reliably include drones.
            if YOLO is not None:
                import os
                base_dir=os.path.dirname(os.path.abspath(__file__))
                model_path=next((os.path.join(base_dir,name) for name in ('drone.pt','best.pt') if os.path.isfile(os.path.join(base_dir,name))),None)
                if model_path:
                    try:
                        self.drone_model=YOLO(model_path)
                        self.drone_model_status='READY'
                    except Exception as model_exc:
                        self.drone_model_status='MODEL ERROR'
                        print(f'[Drone detection] Gagal memuat model {model_path}: {model_exc}')
                else:
                    self.drone_model_status='ADD drone.pt'
            else:
                self.drone_model_status='INSTALL ULTRALYTICS'
            self.face_count_label.config(text='PERSONS: 0  |  DRONES: 0',fg=GREEN)
            self.camera_status.config(text=f'CAMERA STATUS: CONNECTING… • PERSON DETECTION ON • DRONE MODEL: {self.drone_model_status}',fg=AMBER)
            self.camera_thread=threading.Thread(target=self._camera_reader,args=(cap,),daemon=True)
            self.camera_thread.start()
            self._camera_render()
        except Exception as exc:
            self.camera_running=False
            self.camera_status.config(text='CAMERA STATUS: ERROR',fg=RED)
            self.camera_view.config(image='',text='CAMERA UNAVAILABLE\n\nCheck camera source, permissions, or stream URL',fg=RED)
            messagebox.showerror('Live camera error',str(exc))

    def _camera_reader(self, cap):
        while self.camera_running and self.alive:
            ok, frame=cap.read()
            if not ok:
                time.sleep(0.08)
                continue
            try:
                # Person/face detection and approximate distance (green overlay).
                faces=[]
                if self.face_detection_enabled and self.face_cascade is not None:
                    small=cv2.resize(frame,None,fx=0.5,fy=0.5,interpolation=cv2.INTER_AREA)
                    gray=cv2.cvtColor(small,cv2.COLOR_BGR2GRAY)
                    faces=self.face_cascade.detectMultiScale(gray,scaleFactor=1.12,minNeighbors=5,minSize=(24,24))
                    frame_h, frame_w = frame.shape[:2]
                    face_real_height_m = 0.23
                    horizontal_fov_deg = 70.0
                    focal_px = frame_w / (2.0 * math.tan(math.radians(horizontal_fov_deg / 2.0)))
                    for (x,y,w,h) in faces:
                        x,y,w,h=int(x*2),int(y*2),int(w*2),int(h*2)
                        distance_m=(face_real_height_m*focal_px)/max(h,1)
                        cv2.rectangle(frame,(x,y),(x+w,y+h),(70,255,90),2)
                        cv2.putText(frame,f'PERSON ~{distance_m:.1f} m',(x,max(22,y-8)),cv2.FONT_HERSHEY_SIMPLEX,0.58,(70,255,90),2,cv2.LINE_AA)
                self.face_count=len(faces)

                # Optional custom YOLO model trained to recognize drones.
                # Model output follows the reference style: Drone 0.92 + green box.
                drone_count=0
                if self.drone_model is not None:
                    results=self.drone_model.predict(frame,conf=0.25, imgsz=640, verbose=False)
                    for result in results:
                        names=result.names or {}
                        boxes=getattr(result,'boxes',None)
                        if boxes is None:
                            continue
                        for box in boxes:
                            cls_id=int(box.cls[0].item()) if box.cls is not None else -1
                            class_name=str(names.get(cls_id,cls_id)) if isinstance(names,dict) else str(cls_id)
                            # Accept explicit drone labels; a single-class custom model is also accepted.
                            is_single_class=(len(names)==1) if hasattr(names,'__len__') else False
                            if 'drone' not in class_name.lower() and not is_single_class:
                                continue
                            conf=float(box.conf[0].item()) if box.conf is not None else 0.0
                            x1,y1,x2,y2=[int(v) for v in box.xyxy[0].tolist()]
                            cv2.rectangle(frame,(x1,y1),(x2,y2),(70,255,90),2)
                            label=f'Drone {conf:.2f}'
                            cv2.putText(frame,label,(x1,max(22,y1-8)),cv2.FONT_HERSHEY_SIMPLEX,0.62,(70,255,90),2,cv2.LINE_AA)
                            drone_count+=1
                self.drone_count=drone_count
            except Exception as exc:
                # Keep camera feed alive even if a detection frame fails.
                if self.drone_model is not None:
                    self.drone_model_status='INFERENCE ERROR'
                print(f'[Camera detection] {exc}')
            with self.camera_lock:
                self.camera_frame=frame
        try:
            cap.release()
        except Exception:
            pass

    def _camera_render(self):
        if not self.camera_running or not self.alive:
            return
        frame=None
        with self.camera_lock:
            if self.camera_frame is not None:
                frame=self.camera_frame.copy()
        if frame is not None and self.camera_view.winfo_exists():
            try:
                width=max(self.camera_view.winfo_width(),320)
                height=max(self.camera_view.winfo_height(),180)
                frame=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
                h,w=frame.shape[:2]
                scale=min(width/max(w,1),height/max(h,1))
                new_size=(max(1,int(w*scale)),max(1,int(h*scale)))
                frame=cv2.resize(frame,new_size,interpolation=cv2.INTER_AREA)
                image=Image.fromarray(frame)
                photo=ImageTk.PhotoImage(image=image)
                self.camera_photo=photo
                self.camera_view.config(image=photo,text='')
                source_label='USB CAMERA' if self.camera_source_type.get() == 'USB WEBCAM' else 'DRONE STREAM'
                self.face_count_label.config(text=f'PERSONS: {self.face_count}  |  DRONES: {self.drone_count if self.drone_model is not None else "--"}',fg=GREEN if (self.face_count or self.drone_count) else AMBER)
                if self.drone_model is not None:
                    drone_status=f'DRONE DETECTION ON • {self.drone_count} DRONE(S)'
                else:
                    drone_status=f'DRONE DETECTION UNAVAILABLE • {self.drone_model_status}'
                status=f'CAMERA STATUS: LIVE • {source_label} • PERSON DETECTION ON • {drone_status}'
                self.camera_status.config(text=status,fg=GREEN)
            except Exception:
                pass
        self.camera_after=self.root.after(33,self._camera_render)

    def stop_camera(self, update_status=True):
        self.camera_running=False
        if self.camera_after is not None:
            try:
                self.root.after_cancel(self.camera_after)
            except Exception:
                pass
            self.camera_after=None
        cap=self.camera_capture
        self.camera_capture=None
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass
        self.camera_frame=None
        self.camera_photo=None
        self.face_count=0
        self.drone_count=0
        self.drone_model=None
        if hasattr(self,'camera_view') and self.camera_view.winfo_exists():
            self.camera_view.config(image='',text='LIVE CAMERA OFFLINE\n\nSelect source and press START CAMERA',fg=CYAN)
        if hasattr(self,'face_count_label') and self.face_count_label.winfo_exists():
            self.face_count_label.config(text='PERSONS: --  |  DRONES: --',fg=MUTED)
        if update_status and hasattr(self,'camera_status'):
            self.camera_status.config(text='CAMERA STATUS: OFFLINE',fg=MUTED)

    def draw_location_track(self):
        """Draw a compact GPS trail map styled like the former Mission & GPS panel."""
        canvas=getattr(self,'location_canvas',None)
        if canvas is None or not canvas.winfo_exists():
            return
        canvas.delete('all')
        w=max(canvas.winfo_width(),280)
        h=max(canvas.winfo_height(),125)
        bg='#081321'
        canvas.configure(bg=bg)
        # Map-style coordinate grid, matching the Mission & GPS visual.
        for x in range(0,w,28):
            canvas.create_line(x,0,x,h,fill='#14253a',width=1)
        for y in range(0,h,24):
            canvas.create_line(0,y,w,y,fill='#14253a',width=1)
        canvas.create_text(10,7,anchor='nw',text='GPS TRACK / LOCAL RELATIVE VIEW',
                           fill=CYAN,font=('Consolas',7,'bold'))
        raw_points=list(self.location_history[-80:])
        if len(raw_points)<2:
            # Temporary visual preview until GPS samples accumulate.
            preview=[(0.08,0.70),(0.17,0.63),(0.27,0.72),(0.37,0.45),
                     (0.49,0.52),(0.60,0.30),(0.72,0.38),(0.84,0.22),(0.93,0.31)]
            xy=[(18+x*(w-36),24+y*(h-48)) for x,y in preview]
        else:
            lat0=sum(p[0] for p in raw_points)/len(raw_points)
            # Approximate local meters so latitude/longitude can be fitted without
            # repeatedly normalizing and corrupting the stored GPS history.
            cos_lat=max(0.15,math.cos(math.radians(lat0)))
            # Use longitude as east and latitude as north.
            meters=[((lon-(sum(p[1] for p in raw_points)/len(raw_points)))*111320*cos_lat,
                     (lat-lat0)*111320) for lat,lon in raw_points]
            xs=[p[0] for p in meters]; ys=[p[1] for p in meters]
            minx,maxx=min(xs),max(xs); miny,maxy=min(ys),max(ys)
            dx=max(maxx-minx,8.0); dy=max(maxy-miny,8.0)
            pad_x=22; pad_top=25; pad_bottom=20
            scale=min((w-2*pad_x)/dx,(h-pad_top-pad_bottom)/dy)
            cx=(minx+maxx)/2; cy=(miny+maxy)/2
            xy=[(w/2+(x-cx)*scale, h/2-(y-cy)*scale+4) for x,y in meters]
        if len(xy)>=2:
            flat=[v for point in xy for v in point]
            canvas.create_line(*flat,fill='#174c67',width=7,capstyle='round',joinstyle='round')
            canvas.create_line(*flat,fill=CYAN,width=2,capstyle='round',joinstyle='round')
            for px,py in xy[::max(1,len(xy)//12)]:
                canvas.create_oval(px-2,py-2,px+2,py+2,fill='#7be7ff',outline='')
            x,y=xy[-1]
            canvas.create_oval(x-5,y-5,x+5,y+5,fill=GREEN,outline='#e5fff5',width=1)
            canvas.create_text(min(max(x+9,44),w-8),max(y-9,20),anchor='w',
                               text='DRONE',fill=GREEN,font=('Consolas',7,'bold'))
        canvas.create_text(10,h-7,anchor='sw',
                           text=f"LAT {self.d['lat']:.6f}   LON {self.d['lon']:.6f}",
                           fill=MUTED,font=('Consolas',7))

    def _show_location_map(self):
        # Mission & GPS tab is removed; open the current coordinates in the browser map.
        try:
            lat=float(self.d['lat']); lon=float(self.d['lon'])
            webbrowser.open(f'https://www.openstreetmap.org/?mlat={lat}&mlon={lon}#map=16/{lat}/{lon}')
        except Exception as exc:
            messagebox.showerror('Open Map', f'Could not open the map: {exc}')

    def _make_map(self, parent, height=240, compact=False):
        if tkintermapview is not None:
            widget=tkintermapview.TkinterMapView(parent, height=height, corner_radius=0)
            widget.set_position(self.d['lat'], self.d['lon'])
            widget.set_zoom(15 if compact else 16)
            widget.set_tile_server('https://a.tile.openstreetmap.org/{z}/{x}/{y}.png', max_zoom=19)
            widget.configure(bg='#081321')
            return widget
        fallback=tk.Canvas(parent,bg='#081321',height=height,highlightthickness=0)
        fallback.create_text(12,12,anchor='nw',text='MAP LIBRARY UNAVAILABLE',fill=CYAN,font=('Consolas',9,'bold'))
        fallback.create_text(12,34,anchor='nw',text='Install dependencies with RUN_AERONIQ.bat',fill=TEXT,font=('Segoe UI',9))
        fallback.create_text(12,56,anchor='nw',text='OpenStreetMap requires an internet connection.',fill=MUTED,font=('Segoe UI',8))
        return fallback

    def _update_real_maps(self):
        if tkintermapview is None: return
        now=time.time()
        if now-self.last_map_refresh < 1.0: return
        self.last_map_refresh=now
        pos=(float(self.d['lat']),float(self.d['lon']))
        points=list(self.track)
        for attr_marker,attr_path,widget_name in [
            ('map_marker_over','map_path_over','overview_map'),
            ('map_marker_mission','map_path_mission','map')]:
            widget=getattr(self,widget_name,None)
            if widget is None or not hasattr(widget,'set_marker'): continue
            marker=getattr(self,attr_marker)
            if marker is None:
                marker=widget.set_marker(pos[0],pos[1],text='UAV / CURRENT POSITION')
                setattr(self,attr_marker,marker)
            else:
                marker.set_position(pos[0],pos[1])
            path=getattr(self,attr_path)
            if len(points)>=2:
                if path is None:
                    path=widget.set_path(points,color=CYAN,width=3)
                    setattr(self,attr_path,path)
                else:
                    path.set_position_list(points)
            # Keep the current drone visible without changing zoom every update.
            if self.demo and getattr(self, f'_centered_{widget_name}', False) is False:
                widget.set_position(pos[0],pos[1])
                setattr(self, f'_centered_{widget_name}', True)

    def draw_overview_map(self):
        if tkintermapview is not None:
            self._update_real_maps()
            return
        if not hasattr(self,'overview_map'): return
        c=self.overview_map; c.delete('all')
        w=max(c.winfo_width(),260); h=max(c.winfo_height(),130)
        c.create_text(10,8,anchor='nw',text='GPS / LOCAL MISSION TRACK (MAP OFFLINE)',fill=CYAN,font=('Consolas',8,'bold'))
        cx,cy=w*0.52,h*0.56
        for r in (24,48,72): c.create_oval(cx-r,cy-r,cx+r,cy+r,outline='#183248',width=1)
        c.create_line(cx-90,cy,cx+90,cy,fill='#183248'); c.create_line(cx,cy-65,cx,cy+65,fill='#183248')
        pts=list(self.track)
        if len(pts)>1:
            lat0=pts[0][0]; lon0=pts[0][1]; coords=[]
            for lat,lon in pts: coords.extend((cx+(lon-lon0)*450000,cy-(lat-lat0)*450000))
            if len(coords)>=4: c.create_line(*coords,fill=CYAN,width=2,smooth=True)
        c.create_oval(cx-5,cy-5,cx+5,cy+5,fill=GREEN,outline='')
        c.create_text(w-8,h-8,anchor='se',text='LIVE • GNSS',fill=GREEN,font=('Consolas',7,'bold'))
    def _imu(self):
        # One combined live graph with distinct X/Y/Z colors.
        intro=tk.Frame(self.tab_imu,bg=BG); intro.pack(fill='x',padx=12,pady=(12,6))
        tk.Label(intro,text='IMU / GYROSCOPE',bg=BG,fg=TEXT,font=('Segoe UI',15,'bold')).pack(anchor='w')
        tk.Label(intro,text='ANGULAR RATE  ·  DEGREES PER SECOND (°/s)  ·  LIVE STREAM',bg=BG,fg=MUTED,font=('Consolas',9)).pack(anchor='w',pady=(4,0))
        legend=tk.Frame(self.tab_imu,bg=BG); legend.pack(fill='x',padx=12,pady=(4,6))
        self.gyro_values={}
        for key,label,color in [('gx','X / ROLL','#ff617c'),('gy','Y / PITCH','#55efb0'),('gz','Z / YAW','#438cff')]:
            item=tk.Frame(legend,bg=PANEL,highlightbackground=EDGE,highlightthickness=1); item.pack(side='left',fill='x',expand=True,padx=(0,7))
            tk.Label(item,text='━',bg=PANEL,fg=color,font=('Segoe UI',16,'bold')).pack(side='left',padx=(10,5),pady=6)
            tk.Label(item,text=label,bg=PANEL,fg=color,font=('Consolas',9,'bold')).pack(side='left')
            val=tk.Label(item,text='+0.0 °/s',bg=PANEL,fg=TEXT,font=('Consolas',11,'bold')); val.pack(side='right',padx=10)
            self.gyro_values[key]=val
        graph_panel=self.panel(self.tab_imu,'Combined angular rate graph','X RED / Y GREEN / Z BLUE')
        graph_panel.pack(fill='both',expand=True,padx=10,pady=(0,8))
        self.imu_canvas=tk.Canvas(graph_panel,bg='#07111d',highlightthickness=0,height=390)
        self.imu_canvas.pack(fill='both',expand=True,padx=10,pady=10)
        self.imu_canvas.bind('<Configure>',lambda e:self.draw_imu())
        note=tk.Label(self.tab_imu,text='Demo Mode uses simulated values. Connected mode reads angular rates from MAVLink ATTITUDE. Monitoring only; not a certified flight instrument.',bg=BG,fg=MUTED,font=('Segoe UI',9),wraplength=1200,justify='left')
        note.pack(anchor='w',padx=14,pady=(0,8))
    def draw_imu(self, target=None):
        c = target if target is not None else getattr(self,'imu_canvas',None)
        if c is None: return
        c.delete('all')
        w=max(c.winfo_width(),500); h=max(c.winfo_height(),180)
        # Keep a 60-second rolling window (240 samples at 4 Hz) and label
        # the horizontal axis at 10-second intervals.
        left,right,top,bottom=58,w-18,24,h-48
        vals=[v for k in ('gx','gy','gz') for v in self.gyro_hist[k]]
        peak=max(30.0,max((abs(v) for v in vals),default=0)*1.15)
        c.create_text(left,8,anchor='w',text='ANGULAR VELOCITY (°/s)',fill=CYAN,font=('Consolas',9,'bold'))
        for i in range(5):
            y=top+i*(bottom-top)/4
            value=peak-(2*peak)*i/4
            c.create_line(left,y,right,y,fill=EDGE,width=1)
            c.create_text(left-9,y,text=f'{value:+.0f}',anchor='e',fill=MUTED,font=('Consolas',8))
        zero=top+(bottom-top)/2
        c.create_line(left,zero,right,zero,fill='#526982',width=1.3)
        # Vertical time grid: each interval represents 10 seconds of history.
        # The right edge is the newest sample; labels show the age of samples.
        for seconds in (10,20,30,40,50,60):
            x=right-(seconds/60.0)*(right-left)
            c.create_line(x,top,x,bottom,fill='#1a2a3e',width=1)
            c.create_text(x,bottom+10,anchor='n',text=f'{seconds}s',fill=MUTED,font=('Consolas',8))
        colors={'gx':'#ff617c','gy':'#55efb0','gz':'#438cff'}
        for key in ('gx','gy','gz'):
            seq=list(self.gyro_hist[key])
            if len(seq)<2: continue
            coords=[]
            for i,v in enumerate(seq):
                x=left+i*(right-left)/(len(seq)-1)
                y=top+(peak-v)*(bottom-top)/(2*peak)
                # Slight pixel jitter and straight segments match the sharp, live waveform style.
                y += random.uniform(-1.8,1.8)
                coords.extend((x,y))
            c.create_line(*coords,fill=colors[key],width=1.7)
        c.create_text(left,bottom+29,anchor='w',text='60-SECOND HISTORY',fill=MUTED,font=('Consolas',8))
        c.create_text(right,bottom+29,anchor='e',text='NOW →',fill=CYAN,font=('Consolas',8,'bold'))
    def _mission(self):
        row=tk.Frame(self.tab_mission,bg=BG); row.pack(fill='both',expand=True,pady=8); p=self.panel(row,'OpenStreetMap GPS track','SIMULATED / LIVE'); p.pack(side='left',fill='both',expand=True,padx=(0,8)); self.map=self._make_map(p,height=480,compact=False); self.map.pack(fill='both',expand=True,padx=10,pady=10); side=tk.Frame(row,bg=BG,width=340); side.pack(side='left',fill='y'); side.pack_propagate(False)
        p=self.panel(side,'Position solution','GNSS'); p.pack(fill='x',pady=(0,8)); self.gpslabels={}; box=tk.Frame(p,bg=PANEL); box.pack(fill='x',padx=14,pady=12)
        for k,label in [('lat','LATITUDE'),('lon','LONGITUDE'),('sats','SATELLITES'),('heading','HEADING'),('climb','VERTICAL SPEED')]:
            r=tk.Frame(box,bg=PANEL); r.pack(fill='x',pady=5); tk.Label(r,text=label,bg=PANEL,fg=MUTED,font=('Consolas',8)).pack(side='left'); v=tk.Label(r,text='--',bg=PANEL,fg=TEXT,font=('Consolas',9,'bold')); v.pack(side='right'); self.gpslabels[k]=v
        p=self.panel(side,'Mission checklist','OPERATOR NOTES'); p.pack(fill='both',expand=True); self.checks=[]; q=tk.Frame(p,bg=PANEL); q.pack(fill='both',expand=True,padx=12,pady=8)
        for txt in ['Pre-flight inspection complete','Battery and propellers checked','GPS lock confirmed','Telemetry link verified','Landing area reviewed']:
            var=tk.BooleanVar(value=False); self.checks.append(var); tk.Checkbutton(q,text=txt,variable=var,bg=PANEL,fg=TEXT,selectcolor=PANEL2,activebackground=PANEL,activeforeground=CYAN,font=('Segoe UI',9),wraplength=280,anchor='w').pack(fill='x',pady=5)
        tk.Label(q,text='Checklist is advisory only; follow your aircraft manual.',bg=PANEL,fg=AMBER,font=('Segoe UI',8),wraplength=280,justify='left').pack(anchor='w',pady=(12,0))
    def _system(self):
        row=tk.Frame(self.tab_system,bg=BG); row.pack(fill='both',expand=True,pady=8); p=self.panel(row,'Telemetry data recorder','CSV EXPORT'); p.pack(side='left',fill='both',expand=True,padx=(0,8)); c=tk.Frame(p,bg=PANEL); c.pack(fill='x',padx=18,pady=18); tk.Label(c,text='Record live telemetry to a timestamped CSV file for later analysis.',bg=PANEL,fg=MUTED,font=('Segoe UI',10)).pack(anchor='w',pady=(0,12)); self.rec_label=tk.Label(c,text='● NOT RECORDING',bg=PANEL,fg=MUTED,font=('Consolas',10,'bold')); self.rec_label.pack(anchor='w',pady=(0,10)); self.btn(c,'START CSV RECORDING',self.start_log,GREEN,'#06111c').pack(fill='x'); self.btn(c,'STOP AND CLOSE LOG',self.stop_log).pack(fill='x',pady=7)
        p=self.panel(row,'System information','BUILD'); p.pack(side='left',fill='both',expand=True); info=tk.Frame(p,bg=PANEL); info.pack(fill='both',expand=True,padx=18,pady=16)
        for label,value in [('Application','AERONIQ'),('Telemetry protocol','MAVLink (optional)'),('Default mode','Safe demo simulation'),('Camera','USB webcam / RTSP supported (optional)'),('Flight commands','Disabled'),('Data export','CSV')]: tk.Label(info,text=label.upper(),bg=PANEL,fg=MUTED,font=('Consolas',8,'bold')).pack(anchor='w',pady=(6,0)); tk.Label(info,text=value,bg=PANEL,fg=TEXT,font=('Segoe UI',10)).pack(anchor='w')
    def draw_hud(self):
        """Render a larger, more polished stylized 3D quadcopter with animated rotors."""
        c=self.hud
        c.delete('all')
        w=max(c.winfo_width(),400); h=max(c.winfo_height(),220)
        c.create_rectangle(0,0,w,h,fill='#07111d',outline='')
        c.create_text(14,12,anchor='nw',text='3D DRONE / ROTOR STATUS',fill=CYAN,font=('Consolas',9,'bold'))
        c.create_text(14,h-10,anchor='sw',text='PERSPECTIVE MODEL  •  LIVE ATTITUDE  •  ROTORS ANIMATED',fill=MUTED,font=('Consolas',7))

        # Reserve the right side for numeric attitude readouts; enlarge the model
        # and center it within the remaining viewport instead of pinning it to the edge.
        rx=w-145; rw=128
        view_w=max(220,w-170)
        cx=view_w*0.50
        cy=h*0.56
        scale=min(view_w/380.0,(h-28)/185.0,1.18)
        scale=max(scale,0.62)

        roll=math.radians(self.d.get('roll',0.0))
        pitch=math.radians(self.d.get('pitch',0.0))
        yaw=math.radians(self.d.get('heading',0.0))
        elev=math.radians(34.0)
        ca,sa=math.cos(roll),math.sin(roll)
        cb,sb=math.cos(pitch),math.sin(pitch)
        cc,sc=math.cos(yaw),math.sin(yaw)

        def transform(pt):
            x,y,z=pt
            y,z=y*ca-z*sa,y*sa+z*ca
            x,z=x*cb+z*sb,-x*sb+z*cb
            x,y=x*cc-y*sc,x*sc+y*cc
            screen_y=y*math.sin(elev)-z*math.cos(elev)
            depth=y*math.cos(elev)+z*math.sin(elev)
            perspective=1.0/(1.0+depth/650.0)
            return (cx+x*scale*1.22*perspective,
                    cy+screen_y*scale*1.08*perspective,
                    depth)

        def poly3(points,fill,outline='',width=1):
            coords=[]
            for point in points:
                q=transform(point); coords.extend((q[0],q[1]))
            c.create_polygon(*coords,fill=fill,outline=outline or fill,width=width)

        def line3(a,b,color,width=3):
            A=transform(a); B=transform(b)
            c.create_line(A[0],A[1],B[0],B[1],fill=color,
                          width=max(1,int(width*scale)),capstyle='round',joinstyle='round')

        motors=[(-82,-62,0),(82,-62,0),(-82,62,0),(82,62,0)]

        # Subtle ground shadow helps the airframe read as a 3D object.
        shadow=transform((0,0,-15))
        c.create_oval(shadow[0]-105*scale,shadow[1]-12*scale,
                      shadow[0]+105*scale,shadow[1]+12*scale,
                      fill='#0b1a29',outline='')

        # Carbon-fiber arms: dark lower edge, metallic top edge and joint collars.
        for i,(mx,my,mz) in enumerate(motors):
            line3((0,0,-1),(mx,my,mz-1),'#0b1826',15)
            line3((0,0,2),(mx,my,mz+2),'#314b64',11)
            line3((0,0,5),(mx,my,mz+4),'#7590a8',4)
            joint=transform((mx*0.72,my*0.72,4))
            c.create_oval(joint[0]-4*scale,joint[1]-3*scale,
                          joint[0]+4*scale,joint[1]+3*scale,
                          fill='#162d42',outline='#5b7892',width=1)

        # Short landing skids visible under the chassis.
        for yy in (-11,11):
            line3((-20,yy,-10),(-28,yy+3,-22),'#263e52',4)
            line3((20,yy,-10),(28,yy+3,-22),'#263e52',4)
        line3((-28,-8,-22),(28,-5,-22),'#6b8296',3)
        line3((-28,14,-22),(28,17,-22),'#6b8296',3)

        # Motor pods and animated propeller discs.
        spin=self.t*31.0
        for i,(mx,my,mz) in enumerate(motors):
            q=transform((mx,my,mz+7))
            rad=13*scale
            # Soft rotor halo and the motor housing.
            c.create_oval(q[0]-rad*1.65,q[1]-rad*0.82,
                          q[0]+rad*1.65,q[1]+rad*0.82,
                          outline='#1d5267',width=max(1,int(2*scale)))
            c.create_oval(q[0]-rad,q[1]-rad*0.62,
                          q[0]+rad,q[1]+rad*0.62,
                          fill='#13283b',outline='#8aa7bf',width=1)
            c.create_oval(q[0]-5*scale,q[1]-3.2*scale,
                          q[0]+5*scale,q[1]+3.2*scale,
                          fill='#425e76',outline=CYAN,width=1)

            # Two broad, tapered blades rotate continuously; alternating rotor
            # directions are represented with different highlight colors.
            ang=spin*(1 if i in (0,3) else -1)+(i%2)*0.9
            blade_color=('#49d8f4' if i in (0,3) else '#55efb0')
            for offset in (0,math.pi):
                a=ang+offset
                ux,uy=math.cos(a),math.sin(a)
                px,py=-uy,ux
                inner=8; outer=35
                blade=[(mx+ux*inner+px*3.8,my+uy*inner+py*3.8,mz+8),
                       (mx+ux*outer+px*2.0,my+uy*outer+py*2.0,mz+8),
                       (mx+ux*(outer+4)-px*2.0,my+uy*(outer+4)-py*2.0,mz+8),
                       (mx+ux*inner-px*3.8,my+uy*inner-py*3.8,mz+8)]
                poly3(blade,fill=blade_color,outline='#b3f5ff',width=1)
            c.create_text(q[0],q[1]+rad+8*scale,text=f'M{i+1}',
                          fill=MUTED,font=('Consolas',7))

        # Sculpted central fuselage with beveled top, nose, and side walls.
        top_shell=[(-27,-17,8),(-14,-25,10),(15,-23,10),(28,-12,7),
                   (24,16,5),(8,24,6),(-20,18,6),(-28,5,7)]
        lower_shell=[(x*0.88,y*0.88,-7) for x,y,z in top_shell]
        for i in range(len(top_shell)):
            j=(i+1)%len(top_shell)
            poly3([top_shell[i],top_shell[j],lower_shell[j],lower_shell[i]],
                  '#102538',outline='#355a75',width=1)
        poly3(top_shell,'#214b67',outline='#8bdff1',width=2)
        # Top armor panels and center spine add shape and highlights.
        poly3([(-15,-15,11),(0,-21,12),(16,-14,11),(8,-3,12),(-10,-4,12)],
              '#36718c',outline='#5da7bd',width=1)
        poly3([(-10,2,12),(8,0,12),(15,13,8),(0,18,8),(-15,10,8)],
              '#17354d',outline='#46738d',width=1)
        line3((0,-18,13),(0,13,11),'#9defff',2)
        # Nose-mounted camera with lens ring.
        cam=transform((0,-25,5))
        c.create_oval(cam[0]-7*scale,cam[1]-5*scale,
                      cam[0]+7*scale,cam[1]+5*scale,
                      fill='#101820',outline='#9ab6c9',width=1)
        c.create_oval(cam[0]-3.2*scale,cam[1]-3.2*scale,
                      cam[0]+3.2*scale,cam[1]+3.2*scale,
                      fill='#36d9ff',outline='#d6faff',width=1)
        # Status LEDs: red on port side, green on starboard side.
        led_l=transform((-17,4,12)); led_r=transform((17,4,12))
        for q,col in ((led_l,'#ff617c'),(led_r,'#55efb0')):
            c.create_oval(q[0]-3*scale,q[1]-3*scale,q[0]+3*scale,q[1]+3*scale,
                          fill=col,outline='#d8faff',width=1)

        # Small XYZ orientation gizmo, kept close to the fuselage.
        A=transform((0,0,17))
        for endpoint,color,label in [((34,0,17),'#ff617c','X'),
                                     ((0,34,17),'#55efb0','Y'),
                                     ((0,0,48),'#438cff','Z')]:
            B=transform(endpoint)
            c.create_line(A[0],A[1],B[0],B[1],fill=color,width=2,arrow=tk.LAST)
            c.create_text(B[0]+(5 if label!='Y' else -5),B[1]-5,
                          text=label,fill=color,font=('Consolas',8,'bold'))

        # Attitude readouts remain aligned in a dedicated right-side block.
        c.create_text(rx,32,anchor='nw',text='FLIGHT ATTITUDE',
                      fill=CYAN,font=('Consolas',8,'bold'))
        readouts=[('Roll',self.d.get('roll',0.0),'#ff617c'),
                  ('Pitch',self.d.get('pitch',0.0),'#55efb0'),
                  ('Yaw',self.d.get('heading',0.0),'#438cff')]
        for i,(label,value,color) in enumerate(readouts):
            yy=52+i*35
            c.create_rectangle(rx,yy,rx+rw,yy+28,fill=PANEL2,outline=EDGE)
            c.create_text(rx+9,yy+14,anchor='w',text=label,fill=MUTED,font=('Segoe UI',9))
            shown=f'{value:+.1f}°' if label!='Yaw' else f'{value%360:.1f}°'
            c.create_text(rx+rw-8,yy+14,anchor='e',text=shown,
                          fill=TEXT,font=('Consolas',9,'bold'))
        c.create_text(rx,yy+42,anchor='nw',text='ROTOR STATUS: SPINNING',
                      fill=GREEN,font=('Consolas',7,'bold'))

    def draw_chart(self,k):
        """Draw a technical-indicator style chart: raw signal + fast/slow EMA.
        The three colored traces share one time axis and use a 60-second window.
        """
        c,_base_color=self.charts[k]
        c.delete('all')
        w=max(c.winfo_width(),260); h=max(c.winfo_height(),110)
        vals=list(self.hist[k])
        if len(vals)<2: return
        left,right,top,bottom=34,w-10,19,h-34
        # Calculate fast and slow exponential moving averages from the same samples.
        def ema(data, period):
            alpha=2.0/(period+1.0)
            out=[]
            current=float(data[0])
            for value in data:
                current=alpha*float(value)+(1-alpha)*current
                out.append(current)
            return out
        fast=ema(vals,20)   # 5 seconds at 4 Hz
        slow=ema(vals,60)   # 15 seconds at 4 Hz
        series=[(vals,'#ff617c'),(fast,'#55efb0'),(slow,'#438cff')]
        all_values=[v for seq,_ in series for v in seq]
        lo=min(all_values); hi=max(all_values)
        spread=hi-lo
        if spread<0.2:
            center=(hi+lo)/2; lo=center-0.5; hi=center+0.5
        else:
            pad=spread*0.12; lo-=pad; hi+=pad
        # Horizontal grid and axis labels.
        for i in range(5):
            y=top+i*(bottom-top)/4
            value=hi-(hi-lo)*i/4
            c.create_line(left,y,right,y,fill=EDGE,width=1)
            c.create_text(left-5,y,text=f'{value:.1f}',anchor='e',fill=MUTED,font=('Consolas',7))
        # Vertical grid is marked in seconds; newest sample is at the right edge.
        for seconds in (10,20,30,40,50,60):
            x=right-(seconds/60.0)*(right-left)
            c.create_line(x,top,x,bottom,fill='#1a2a3e',width=1)
            c.create_text(x,bottom+11,text=f'{seconds}s',anchor='n',fill=MUTED,font=('Consolas',7))
        for seq,color in series:
            coords=[]
            for i,value in enumerate(seq):
                x=left+i*(right-left)/(len(seq)-1)
                y=top+(hi-value)*(bottom-top)/(hi-lo)
                coords.extend((x,y))
            c.create_line(*coords,fill=color,width=1.5,smooth=False)
        # Compact current-value key, colored to match the combined technical graph.
        unit='m' if k=='alt' else 'm/s'
        c.create_text(left,5,anchor='w',text=f'RAW {vals[-1]:.1f} {unit}',fill='#ff617c',font=('Consolas',8,'bold'))
        c.create_text(right,bottom+25,anchor='e',text='60s HISTORY  ←                         NOW →',fill=MUTED,font=('Consolas',7))
    def draw_map(self):
        if tkintermapview is not None:
            self._update_real_maps()
            return
        c=self.map; c.delete('all'); w=max(c.winfo_width(),400); h=max(c.winfo_height(),300)
        for x in range(0,w,36): c.create_line(x,0,x,h,fill='#14253a')
        for y in range(0,h,36): c.create_line(0,y,w,y,fill='#14253a')
        c.create_text(14,14,anchor='nw',text='GPS TRACK / LOCAL RELATIVE VIEW (MAP OFFLINE)',fill=CYAN,font=('Consolas',9,'bold'))
        pts=list(self.track)
        if len(pts)>1:
            lat0=sum(p[0] for p in pts)/len(pts); lon0=sum(p[1] for p in pts)/len(pts); xy=[]
            for lat,lon in pts: xy += [w/2+(lon-lon0)*700000,h/2-(lat-lat0)*700000]
            if len(xy)>=4: c.create_line(*xy,fill=CYAN,width=2,smooth=True)
            c.create_oval(xy[-2]-6,xy[-1]-6,xy[-2]+6,xy[-1]+6,fill=GREEN,outline='')
        c.create_text(14,h-14,anchor='sw',text=f"LAT {self.d['lat']:.6f}   LON {self.d['lon']:.6f}",fill=MUTED,font=('Consolas',8))
    def update_alerts(self):
        items=[]
        if self.d['battery']<25: items.append(('CRITICAL','Battery below 25%'))
        elif self.d['battery']<40: items.append(('WARNING','Battery below 40%'))
        if self.d['sats']<6: items.append(('WARNING','Low satellite count'))
        if self.d.get('rssi',0)<25: items.append(('CRITICAL','Telemetry RSSI critically low'))
        elif self.d.get('rssi',0)<45: items.append(('WARNING','Telemetry RSSI is weak'))
        if not (self.demo or self.connected): items.append(('WARNING','Telemetry link unavailable'))
        if not items: items=[('NOMINAL','All monitored values within demo thresholds')]
        msg='\n'.join(f'[{a}] {b}' for a,b in items)
        self.alertbox.config(state='normal'); self.alertbox.delete('1.0','end'); self.alertbox.insert('1.0',msg); self.alertbox.config(state='disabled')
    def draw_mini_telemetry(self):
        c=self.mini_telemetry
        c.delete('all')
        pct=max(0,min(100,float(self.d.get('battery',0))))
        # Battery icon with the aircraft battery percentage displayed inside it.
        battery_color=RED if pct<20 else AMBER if pct<40 else GREEN
        bx1,by1,bx2,by2=7,17,79,48
        c.create_rectangle(bx1,by1,bx2,by2,outline=battery_color,width=2)
        c.create_rectangle(bx2+2,25,bx2+7,40,fill=battery_color,outline=battery_color)
        inner_left,inner_top,inner_right,inner_bottom=11,21,75,44
        fill_right=inner_left+(inner_right-inner_left)*pct/100
        if pct>0:
            c.create_rectangle(inner_left,inner_top,fill_right,inner_bottom,fill=battery_color,outline='')
        # Dark label plate keeps the percentage legible at every battery level.
        c.create_rectangle(25,24,62,41,fill='#07111d',outline='')
        c.create_text(43.5,32.5,text=f'{pct:.0f}%',fill=TEXT,font=('Segoe UI',10,'bold'))
        c.create_text(43,62,text=f"{self.d.get('voltage',0):.1f} V",fill=MUTED,font=('Consolas',8))
        c.create_line(94,8,94,65,fill=EDGE,width=1)
        rssi=self.d.get('rssi',None)
        if rssi is None:
            c.create_text(163,28,text='—',fill=MUTED,font=('Segoe UI',17,'bold'))
            label='RSSI NO DATA'
        else:
            pct_r=max(0,min(100,float(rssi)))
            color=RED if pct_r<40 else AMBER if pct_r<70 else GREEN
            for i,bh in enumerate([9,15,22,30]):
                x=136+i*13
                c.create_rectangle(x,43-bh,x+8,43,fill=color if pct_r >= (i+1)*25 else EDGE,outline='')
            label=f'RSSI  {pct_r:.0f}%'
        c.create_text(163,54,text=label,fill=TEXT,font=('Consolas',9,'bold'))
        raw=self.d.get('rssi_raw',None)
        c.create_text(163,68,text=(f'{raw} RAW' if raw is not None else 'NO SIGNAL DATA'),fill=MUTED,font=('Consolas',8))
    def _tick(self):
        if hasattr(self, 'location_canvas'):
            lat=float(self.d['lat']); lon=float(self.d['lon'])
            if not self.location_history or abs(lat-self.location_history[-1][0])>1e-7 or abs(lon-self.location_history[-1][1])>1e-7:
                self.location_history.append((lat,lon))
                self.location_history=self.location_history[-80:]
            # Keep raw latitude/longitude samples; the drawing routine projects them to screen pixels.
            # GPS status badge is hidden; coordinates remain on the track canvas.
            self.draw_location_track()

        if not self.alive:return
        self.clock.config(text='SYSTEM TIME '+datetime.now().strftime('%H:%M:%S'))
        if self.demo:
            self.t+=.25
            # Irregular telemetry generator: drift regimes + noise, deliberately non-periodic.
            def noisy(key, noise, low=None, high=None, drift_key=None, drift_scale=0.0):
                # Each signal changes its drift regime at irregular intervals,
                # producing a noisy oscilloscope-like trace without sine-wave cycles.
                deadline_key=f'{drift_key}_next_change' if drift_key else None
                if drift_key and self.t >= self.sim.get(deadline_key, 0.0):
                    self.sim[drift_key]=random.uniform(-drift_scale,drift_scale)
                    self.sim[deadline_key]=self.t+random.uniform(1.2,4.0)
                drift=self.sim.get(drift_key,0.0) if drift_key else 0.0
                value=self.sim[key] + drift + random.gauss(0,noise)
                if low is not None and value < low:
                    value=low + (low-value)*0.20
                    if drift_key: self.sim[drift_key]=abs(drift)*0.25
                if high is not None and value > high:
                    value=high - (value-high)*0.20
                    if drift_key: self.sim[drift_key]=-abs(drift)*0.25
                if low is not None: value=max(low,value)
                if high is not None: value=min(high,value)
                self.sim[key]=value
                return value
            alt=noisy('alt',.34,0,60,'alt_drift',.16)
            speed=noisy('speed',.22,0,30,'speed_drift',.10)
            gx=noisy('gx',2.0,-30,30,'gx_drift',.65)
            gy=noisy('gy',1.7,-30,30,'gy_drift',.50)
            gz=noisy('gz',1.5,-30,30,'gz_drift',.45)
            roll=noisy('roll',1.3,-35,35)
            pitch=noisy('pitch',1.1,-35,35)
            rssi=noisy('rssi',1.1,5,100)
            self.d.update(alt=alt,speed=speed,battery=max(0,86-self.t*.018),
                voltage=13.8+self.d['battery']*.019,heading=(self.d['heading']+random.uniform(-1.0,1.4))%360,
                roll=roll,pitch=pitch,
                lat=self.d['lat']+random.uniform(-.000008,.000008),
                lon=self.d['lon']+random.uniform(-.000010,.000010),
                climb=random.uniform(-.45,.45),sats=max(7,min(16,int(round(12+random.gauss(0,1))))),
                rssi=rssi,rssi_raw=int(max(0,min(254,(rssi/100)*254))),gx=gx,gy=gy,gz=gz)
        for k in self.hist:self.hist[k].append(self.d[k])
        self.track.append((self.d['lat'],self.d['lon']))
        for k in ['alt','speed']: self.big[k].config(text=f"{self.d[k]:.1f}")
        self.status['flight'].config(text='ARMED' if self.d['armed'] else 'DISARMED',fg=RED if self.d['armed'] else GREEN); self.status['mode'].config(text=self.d['mode'],fg=CYAN); self.status['gps'].config(text=f"{self.d['sats']} SAT / {self.d['fix']}",fg=GREEN if self.d['sats']>=6 else RED); self.status['link'].config(text='DEMO' if self.demo else 'CONNECTED' if self.connected else 'OFFLINE',fg=GREEN if (self.demo or self.connected) else RED)
        for k,fmt in [('lat',lambda x:f'{x:.6f}'),('lon',lambda x:f'{x:.6f}'),('sats',str),('heading',lambda x:f'{x:.0f}°'),('climb',lambda x:f'{x:.1f} m/s')]: self.gpslabels[k].config(text=fmt(self.d[k]))
        if hasattr(self,'overview_gpslabels'):
            for k,fmt in [('lat',lambda x:f'{x:.5f}'),('lon',lambda x:f'{x:.5f}'),('sats',str),('heading',lambda x:f'{x:.0f}°')]: self.overview_gpslabels[k].config(text=fmt(self.d[k]))
        for axis in ('gx','gy','gz'):
            self.gyro_values[axis].config(text=f"{self.d[axis]:+.1f} °/s")
            self.gyro_hist[axis].append(self.d[axis])
        self.draw_hud(); self.draw_map(); self.draw_overview_map(); self._update_real_maps(); self.draw_mini_telemetry(); self.draw_imu()
        for k in self.charts:self.draw_chart(k)
        if hasattr(self,'imu_mini_canvas'): self.draw_imu(self.imu_mini_canvas)
        self.update_alerts()
        if self.writer:
            try:self.writer.writerow([datetime.now().isoformat(timespec='seconds')]+[self.d[k] for k in ['alt','speed','battery','voltage','heading','roll','pitch','lat','lon','sats','mode','armed','rssi','rssi_raw','gx','gy','gz']]); self.log.flush()
            except Exception:pass
        self.foot.config(text='DEMO SIMULATION' if self.demo else 'MAVLINK CONNECTED' if self.connected else 'LINK OFFLINE',fg=GREEN if self.demo or self.connected else RED)
        self.root.after(250,self._tick)
    def connect(self):
        if not mavutil: messagebox.showerror('Dependency missing','Install pymavlink with: python -m pip install pymavlink'); return
        ep=self.endpoint.get().strip(); typ=self.transport.get()
        try:
            conn='udpin:'+ep if typ=='UDP' else 'tcp:'+ep if typ=='TCP' else ep; self.link=mavutil.mavlink_connection(conn,baud=57600) if typ=='Serial' else mavutil.mavlink_connection(conn); self.demo=False; self.badge.config(text='● WAITING FOR MAVLINK',bg='#342b1c',fg=AMBER); threading.Thread(target=self.reader,daemon=True).start()
        except Exception as e:messagebox.showerror('Connection failed',str(e))
    def reader(self):
        try:
            self.link.wait_heartbeat(timeout=10); self.connected=True; self.root.after(0,lambda:self.badge.config(text='● MAVLINK LINK ACTIVE',bg='#12302e',fg=GREEN))
            while self.alive and self.link:
                m=self.link.recv_match(blocking=True,timeout=1)
                if not m:continue
                typ=m.get_type(); d=self.d
                if typ=='GLOBAL_POSITION_INT':d.update(lat=m.lat/1e7,lon=m.lon/1e7,alt=m.relative_alt/1000,heading=m.hdg/100 if m.hdg!=65535 else d['heading'],climb=-m.vz/100)
                elif typ=='VFR_HUD':d.update(speed=float(m.groundspeed),alt=float(m.alt),heading=float(m.heading),climb=float(m.climb))
                elif typ=='ATTITUDE':d.update(roll=math.degrees(m.roll),pitch=math.degrees(m.pitch),gx=math.degrees(m.rollspeed),gy=math.degrees(m.pitchspeed),gz=math.degrees(m.yawspeed))
                elif typ=='SYS_STATUS':
                    if m.voltage_battery!=65535:d['voltage']=m.voltage_battery/1000
                    if m.battery_remaining>=0:d['battery']=float(m.battery_remaining)
                elif typ=='GPS_RAW_INT':d.update(sats=int(m.satellites_visible),fix='3D FIX' if m.fix_type>=3 else 'NO FIX')
                elif typ=='RADIO_STATUS':
                    # MAVLink RADIO_STATUS RSSI commonly uses 0-254; 255 means unknown.
                    raw=int(getattr(m,'rssi',255)); remote=int(getattr(m,'remrssi',255))
                    sample=remote if remote not in (255,0) else raw
                    if sample not in (255,): d.update(rssi=max(0,min(100,sample/254*100)),rssi_raw=sample)
                elif typ=='HEARTBEAT':
                    d['mode']=mavutil.mode_string_v10(m); d['armed']=bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        except Exception:self.root.after(0,lambda:self.foot.config(text='MAVLINK LINK ERROR',fg=RED))
        finally:self.connected=False
    def demo_mode(self):
        try:
            if self.link:self.link.close()
        except Exception:pass
        self.link=None; self.connected=False; self.demo=True; self.badge.config(text='● DEMO SIMULATION',bg='#12302e',fg=GREEN)
    def start_log(self):
        if self.writer:return
        path=filedialog.asksaveasfilename(defaultextension='.csv',filetypes=[('CSV files','*.csv')],initialfile='AERONIQ.csv')
        if not path:return
        try:self.log=open(path,'w',newline='',encoding='utf-8'); self.writer=csv.writer(self.log); self.writer.writerow(['timestamp','altitude_m','ground_speed_mps','battery_pct','voltage_v','heading_deg','roll_deg','pitch_deg','latitude','longitude','satellites','flight_mode','armed','rssi_percent','rssi_raw','gyro_x_deg_s','gyro_y_deg_s','gyro_z_deg_s']); self.rec_label.config(text='● RECORDING TELEMETRY',fg=RED)
        except Exception as e:messagebox.showerror('CSV error',str(e))
    def stop_log(self):
        if self.log:
            try:self.log.close()
            except Exception:pass
        self.log=None; self.writer=None
        if hasattr(self,'rec_label'):self.rec_label.config(text='● NOT RECORDING',fg=MUTED)
    def close(self):
        self.stop_camera(update_status=False)
        self.alive=False; self.stop_log()
        try:
            if self.link:self.link.close()
        except Exception:pass
        self.root.destroy()

if __name__=='__main__':
    root=tk.Tk(); app=AERONIQ(root); root.mainloop()
