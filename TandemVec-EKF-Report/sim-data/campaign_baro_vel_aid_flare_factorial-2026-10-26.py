import concurrent.futures as cf, json, math, os, re, subprocess
TMP=r"C:\Users\12631\AppData\Local\Temp"; EXE=TMP+r"\camp_aidflare\fw_real.exe"
OUT=TMP+r"\campaign_aidflare"
DIRS={"H":(0,0),"N":(1,0),"E":(0,1),"S":(-1,0),"W":(0,-1),"NE":(1,1)}
JSON_RE=re.compile(r"SIL_REAL_JSON (\{.*\})")
def one(t):
    aid,flare,dv,seed=t
    dn,de=DIRS[dv]; n=math.hypot(dn,de) or 1.0
    ln,le=10*dn/n,10*de/n
    cmd=[EXE,"--real-nav","--profile","mag_clean","--pos-hold","--pos-hold-at","41",
         "--pos-leg",f"{ln:.3f}:{le:.3f}","--gnss-outage","43:103","--alt","30",
         "--seed",str(seed),"--seconds","133","--wind-window","43:103","--wind-force","1.0",
         "--baro-vel-aid",str(aid),"--flare",str(flare)]
    p=subprocess.run(cmd,capture_output=True,text=True,timeout=320)
    m=JSON_RE.findall(p.stdout+p.stderr)
    if not m: return dict(aid=aid,flare=flare,dir=dv,seed=seed,no_json=True)
    d=json.loads(m[-1].replace("nan","NaN"))
    o=dict(aid=aid,flare=flare,dir=dv,seed=seed)
    for k in ("land_t","land_hold","land_vz_down","max_roll_preland","final_alt",
              "baro_vel_attempts","baro_vel_fused"):
        o[k]=d.get(k)
    return o
def main():
    os.makedirs(OUT,exist_ok=True)
    tasks=[(aid,flare,dv,s) for aid in (0,1) for flare in (0,1) for dv in DIRS for s in range(1,11)]
    print("tasks",len(tasks),flush=True)
    with open(os.path.join(OUT,"results.jsonl"),"w",encoding="utf-8") as f, cf.ProcessPoolExecutor(max_workers=6) as ex:
        for r in ex.map(one,tasks,chunksize=2):
            f.write(json.dumps(r)+"\n"); f.flush()
    print("done",flush=True)
if __name__=="__main__": main()
