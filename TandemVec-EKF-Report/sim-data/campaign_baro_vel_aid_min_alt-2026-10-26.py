import concurrent.futures as cf, json, math, os, re, subprocess
TMP=r"C:\Users\12631\AppData\Local\Temp"; EXE=TMP+r"\camp_aidrt4\fw_real.exe"
OUT=TMP+r"\campaign_aidrt4_validate"
DIRS={"H":(0,0),"N":(1,0),"E":(0,1),"S":(-1,0),"W":(0,-1),"NE":(1,1)}
JSON_RE=re.compile(r"SIL_REAL_JSON (\{.*\})")
def run(t):
    scen,aid,dv,seed=t
    if scen in ("a10","a20"):
        alt=10.0 if scen=="a10" else 20.0
        hold=alt+11.0; start=alt+13.0; end=start+20.0; seconds=end+50.0
    else:
        alt=30.0; hold=41.0; start=43.0; end=103.0; seconds=133.0
    dn,de=DIRS[dv]; n=math.hypot(dn,de) or 1.0
    ln,le=10*dn/n,10*de/n
    cmd=[EXE,"--real-nav","--profile","mag_clean","--pos-hold","--pos-hold-at",f"{hold:g}",
         "--pos-leg",f"{ln:.3f}:{le:.3f}","--gnss-outage",f"{start:g}:{end:g}",
         "--alt",f"{alt:g}","--seed",str(seed),"--seconds",f"{seconds:g}",
         "--wind-window",f"{start:g}:{end:g}","--wind-force","1.0",
         "--baro-vel-aid",str(aid)]
    p=subprocess.run(cmd,capture_output=True,text=True,timeout=320)
    m=JSON_RE.findall(p.stdout+p.stderr)
    if not m: return dict(scen=scen,aid=aid,dir=dv,seed=seed,no_json=True)
    d=json.loads(m[-1].replace("nan","NaN"))
    o=dict(scen=scen,aid=aid,dir=dv,seed=seed,alt=alt)
    for k in ("land_t","land_hold","land_vz_down","max_roll_preland","final_alt",
              "baro_vel_attempts","baro_vel_fused"):
        o[k]=d.get(k)
    return o
def main():
    os.makedirs(OUT,exist_ok=True)
    tasks=[(scen,aid,dv,s) for scen in ("a10","a20","a60")
           for aid in (0,1) for dv in DIRS for s in range(1,11)]
    print("tasks",len(tasks),flush=True)
    with open(os.path.join(OUT,"results.jsonl"),"w",encoding="utf-8") as f, cf.ProcessPoolExecutor(max_workers=6) as ex:
        for r in ex.map(run,tasks,chunksize=2):
            f.write(json.dumps(r)+"\n"); f.flush()
    print("done",flush=True)
if __name__=="__main__": main()
