import concurrent.futures as cf, json, math, os, re, subprocess
TMP=r"C:\Users\12631\AppData\Local\Temp"; EXE=TMP+r"\camp_flare8age\fw_real.exe"
OUT=TMP+r"\campaign_flare8age"
DIRS={"H":(0,0),"N":(1,0),"E":(0,1),"S":(-1,0),"W":(0,-1),"NE":(1,1)}
JSON_RE=re.compile(r"SIL_REAL_JSON (\{.*\})")
def run(t):
    dur,dv,seed=t; dn,de=DIRS[dv]; n=math.hypot(dn,de) or 1.0
    ln,le=10*dn/n,10*de/n
    start=43.0; end=start+dur; seconds=end+50.0
    cmd=[EXE,"--real-nav","--profile","mag_clean","--pos-hold","--pos-hold-at","41",
         "--pos-leg",f"{ln:.3f}:{le:.3f}","--gnss-outage",f"43:{end:g}","--alt","30",
         "--seed",str(seed),"--seconds",f"{seconds:g}","--wind-window",f"43:{end:g}","--wind-force","1.0"]
    p=subprocess.run(cmd,capture_output=True,text=True,timeout=320)
    m=JSON_RE.findall(p.stdout+p.stderr)
    if not m: return dict(tag=f"{dur}_{dv}_{seed}",dur=dur,dir=dv,seed=seed,no_json=True)
    d=json.loads(m[-1].replace("nan","NaN"))
    o=dict(tag=f"{dur}_{dv}_{seed}",dur=dur,dir=dv,seed=seed,alt_at_outage=d.get("alt_at_outage"))
    for k in ("land_t","land_hold","land_roll","max_roll_preland","land_vz_down","max_desc_vz_down","final_alt"):
        o[k]=d.get(k)
    o["landed"]=d.get("final_alt") is not None and d["final_alt"]<1.0
    return o
def main():
    os.makedirs(OUT,exist_ok=True)
    tasks=[(dur,dv,s) for dur in (20.0,60.0) for dv in DIRS for s in range(1,11)]
    with open(os.path.join(OUT,"results.jsonl"),"w",encoding="utf-8") as f, cf.ProcessPoolExecutor(max_workers=6) as ex:
        for r in ex.map(run,tasks,chunksize=2):
            f.write(json.dumps(r)+"\n"); f.flush()
    print("done",flush=True)
if __name__=="__main__": main()
