import concurrent.futures as cf, json, math, os, re, subprocess
TMP=r"C:\Users\12631\AppData\Local\Temp"; EXE=TMP+r"\camp_spoofresp\fw_real.exe"
OUT=TMP+r"\campaign_spoof_resp"
DIRS={"H":(0,0),"N":(1,0),"E":(0,1),"S":(-1,0),"W":(0,-1),"NE":(1,1)}
JSON_RE=re.compile(r"SIL_REAL_JSON (\{.*\})")
MODES=[("none",0.0),("step20",20.0),("step30",30.0),("step50",50.0),("step100",100.0)]
def one(t):
    mode,off,resp,dv,seed=t; dn,de=DIRS[dv]; n=math.hypot(dn,de) or 1.0
    ln,le=10*dn/n,10*de/n
    cmd=[EXE,"--real-nav","--profile","mag_clean","--pos-hold","--pos-hold-at","20",
         "--pos-leg",f"{ln:.3f}:{le:.3f}","--seed",str(seed),"--seconds","45","--spoof-response",str(resp)]
    if off>0: cmd += ["--gnss-spoof",f"{off:g}","--gnss-spoof-start","25"]
    p=subprocess.run(cmd,capture_output=True,text=True,timeout=180)
    m=JSON_RE.findall(p.stdout+p.stderr)
    if not m: return dict(mode=mode,resp=resp,dir=dv,seed=seed,no_json=True)
    d=json.loads(m[-1].replace("nan","NaN"))
    o=dict(mode=mode,resp=resp,dir=dv,seed=seed)
    for k in ("pos_rms","pos_max","hold_end_rms","final_alt","max_roll","spoof_det","spoof_deg","outlier_rejects"):
        o[k]=d.get(k)
    return o
def main():
    os.makedirs(OUT,exist_ok=True)
    tasks=[(mode,off,resp,dv,s) for mode,off in MODES for resp in (0,1) for dv in DIRS for s in range(1,11)]
    print("tasks",len(tasks),flush=True)
    with open(os.path.join(OUT,"results.jsonl"),"w",encoding="utf-8") as f, cf.ProcessPoolExecutor(max_workers=6) as ex:
        for r in ex.map(one,tasks,chunksize=2):
            f.write(json.dumps(r)+"\n"); f.flush()
    print("done",flush=True)
if __name__=="__main__": main()
