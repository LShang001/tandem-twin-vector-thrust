import concurrent.futures as cf, json, math, os, re, subprocess
TMP=r"C:\Users\12631\AppData\Local\Temp"; EXE=TMP+r"\camp_baromin\fw_real.exe"
OUT=TMP+r"\campaign_baromin60"
DIRS={"H":(0,0),"N":(1,0),"E":(0,1),"S":(-1,0),"W":(0,-1),"NE":(1,1)}
JSON_RE=re.compile(r"SIL_REAL_JSON (\{.*\})")
def run(task):
    dv,seed=task; dn,de=DIRS[dv]; norm=math.hypot(dn,de) or 1.0
    ln,le=10*dn/norm,10*de/norm
    hold=41.0; start=43.0; end=103.0; seconds=133.0
    cmd=[EXE,"--real-nav","--profile","mag_clean","--pos-hold","--pos-hold-at","41",
         "--pos-leg",f"{ln:.3f}:{le:.3f}","--gnss-outage","43:103","--alt","30",
         "--seed",str(seed),"--seconds","133","--wind-window","43:103","--wind-force","1.0"]
    p=subprocess.run(cmd,capture_output=True,text=True,timeout=320)
    m=JSON_RE.findall(p.stdout+p.stderr)
    if not m: return dict(tag=f"{dv}_s{seed}",dir=dv,seed=seed,no_json=True)
    d=json.loads(m[-1].replace("nan","NaN"))
    o=dict(tag=f"{dv}_s{seed}",dir=dv,seed=seed,alt_at_outage=d.get("alt_at_outage"))
    for k in ("land_t","land_hold","land_roll","max_roll_preland","land_vz_down",
              "max_desc_vz_down","final_alt","pos_rms","pos_max"):
        o[k]=d.get(k)
    o["landed"]=d.get("final_alt") is not None and d["final_alt"]<1.0
    return o
def main():
    os.makedirs(OUT,exist_ok=True)
    tasks=[(dv,s) for dv in DIRS for s in range(1,11)]
    with open(os.path.join(OUT,"results.jsonl"),"w",encoding="utf-8") as f, cf.ProcessPoolExecutor(max_workers=6) as ex:
        for r in ex.map(run,tasks,chunksize=2):
            f.write(json.dumps(r)+"\n"); f.flush()
    print("done",flush=True)
if __name__=="__main__": main()
