import concurrent.futures as cf, json, math, os, re, subprocess
TMP=r"C:\Users\12631\AppData\Local\Temp"; EXE=TMP+r"\camp_aidrt2\fw_real.exe"
OUT=TMP+r"\campaign_aidrt2_validate"
DIRS={"H":(0,0),"N":(1,0),"E":(0,1),"S":(-1,0),"W":(0,-1),"NE":(1,1)}
JSON_RE=re.compile(r"SIL_REAL_JSON (\{.*\})")
def one(t):
    scen,aid,seed,tag_extra=t
    if scen in ("hover20","hover60"):
        dur=20.0 if scen=="hover20" else 60.0
        dn,de=DIRS[tag_extra]; n=math.hypot(dn,de) or 1.0
        ln,le=10*dn/n,10*de/n
        cmd=[EXE,"--real-nav","--profile","mag_clean","--pos-hold","--pos-hold-at","41",
             "--pos-leg",f"{ln:.3f}:{le:.3f}","--gnss-outage",f"43:{43+dur:g}","--alt","30",
             "--seed",str(seed),"--seconds",f"{43+dur+50:g}","--wind-window",f"43:{43+dur:g}",
             "--wind-force","1.0","--baro-vel-aid",str(aid)]
        fields=("pos_rms","pos_max","hold_end_rms","max_roll","final_alt","land_t",
                "land_hold","land_vz_down","max_roll_preland","baro_vel_attempts",
                "baro_vel_fused","baro_vel_rejected")
    else:
        dist=tag_extra; ln=dist/math.sqrt(2); le=dist/math.sqrt(2)
        cmd=[EXE,"--real-nav","--profile","mag_clean","--pos-hold","--pos-hold-at","20",
             "--pos-leg",f"{ln:.3f}:{le:.3f}","--flow","--seed",str(seed),"--seconds","45",
             "--gnss-outage","20:40","--wind-window","20:40","--wind-force","1.0",
             "--baro-vel-aid",str(aid)]
        fields=("pos_rms","pos_max","max_roll","final_alt","baro_vel_attempts",
                "baro_vel_fused","baro_vel_rejected")
    p=subprocess.run(cmd,capture_output=True,text=True,timeout=320)
    m=JSON_RE.findall(p.stdout+p.stderr)
    if not m: return dict(scen=scen,aid=aid,seed=seed,extra=tag_extra,no_json=True)
    d=json.loads(m[-1].replace("nan","NaN"))
    o=dict(scen=scen,aid=aid,seed=seed,extra=tag_extra)
    for k in fields: o[k]=d.get(k)
    o["nav_fail"]=bool((d.get("pos_rms") or 0)>3 or (d.get("pos_max") or 0)>10
                       or abs((d.get("final_alt") or 0)-10)>3 or (d.get("max_roll") or 0)>45)
    o["stab_fail"]=bool(abs((d.get("final_alt") or 0)-10)>3 or (d.get("max_roll") or 0)>45)
    return o
def main():
    os.makedirs(OUT,exist_ok=True)
    tasks=[]
    for scen in ("hover20","hover60"):
        for aid in (0,1):
            for dv in DIRS:
                for s in range(1,11): tasks.append((scen,aid,s,dv))
    for aid in (0,1):
        for dist in (25.0,50.0):
            for s in range(11,31): tasks.append(("ne_fwd",aid,s,dist))
    print("tasks",len(tasks),flush=True)
    with open(os.path.join(OUT,"results.jsonl"),"w",encoding="utf-8") as f, cf.ProcessPoolExecutor(max_workers=6) as ex:
        for r in ex.map(one,tasks,chunksize=2):
            f.write(json.dumps(r)+"\n"); f.flush()
    print("done",flush=True)
if __name__=="__main__": main()
