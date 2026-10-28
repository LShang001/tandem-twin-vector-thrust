import concurrent.futures as cf, json, math, os, re, subprocess
TMP=r"C:\Users\12631\AppData\Local\Temp"; EXE=TMP+r"\camp_aidrt4\fw_real.exe"
OUT=TMP+r"\campaign_aid_duration"
DIRS={"H":(0,0),"N":(1,0),"E":(0,1),"S":(-1,0),"W":(0,-1),"NE":(1,1)}
JSON_RE=re.compile(r"SIL_REAL_JSON (\{.*\})")
def one(t):
    dur,wind,aid,dv,seed=t
    dn,de=DIRS[dv]; n=math.hypot(dn,de) or 1.0
    ln,le=10*dn/n,10*de/n
    start=43.0; end=start+dur; seconds=end+50.0
    cmd=[EXE,"--real-nav","--profile","mag_clean","--pos-hold","--pos-hold-at","41",
         "--pos-leg",f"{ln:.3f}:{le:.3f}","--gnss-outage",f"43:{end:g}","--alt","30",
         "--seed",str(seed),"--seconds",f"{seconds:g}","--baro-vel-aid",str(aid)]
    if wind:
        cmd += ["--wind-window",f"43:{end:g}","--wind-force","1.0"]
    p=subprocess.run(cmd,capture_output=True,text=True,timeout=320)
    m=JSON_RE.findall(p.stdout+p.stderr)
    if not m: return dict(dur=dur,wind=wind,aid=aid,dir=dv,seed=seed,no_json=True)
    d=json.loads(m[-1].replace("nan","NaN"))
    o=dict(dur=dur,wind=wind,aid=aid,dir=dv,seed=seed)
    for k in ("land_t","land_hold","land_vz_down","max_roll_preland","final_alt",
              "pos_rms","pos_max","baro_vel_attempts","baro_vel_fused"):
        o[k]=d.get(k)
    o["nav_fail"]=bool((d.get("pos_rms") or 0)>3 or (d.get("pos_max") or 0)>10)
    return o
def main():
    os.makedirs(OUT,exist_ok=True)
    tasks=[(dur,wind,aid,dv,s) for dur in (10.0,20.0,30.0,60.0) for wind in (0,1)
           for aid in (0,1) for dv in DIRS for s in range(1,11)]
    print("tasks",len(tasks),flush=True)
    with open(os.path.join(OUT,"results.jsonl"),"w",encoding="utf-8") as f, cf.ProcessPoolExecutor(max_workers=6) as ex:
        done=0
        for r in ex.map(one,tasks,chunksize=2):
            f.write(json.dumps(r)+"\n"); f.flush(); done+=1
            if done%100==0: print("done",done,flush=True)
    print("done",flush=True)
if __name__=="__main__": main()
