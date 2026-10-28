"""Settled 30 m + 60 s outage (no recovery before landing): 3 rates x 6 dirs x 10 seeds."""
import concurrent.futures as cf, json, math, os, re, subprocess
TMP=r"C:\Users\12631\AppData\Local\Temp"
ARMS={"r1":TMP+r"\camp_combo6b\fw_real.exe","r2":TMP+r"\camp_rate2\fw_real.exe","r3":TMP+r"\camp_rate3\fw_real.exe"}
OUT=TMP+r"\campaign_alt_rate60"
DIRS={"H":(0,0),"N":(1,0),"E":(0,1),"S":(-1,0),"W":(0,-1),"NE":(1,1)}
JSON_RE=re.compile(r"SIL_REAL_JSON (\{.*\})")
def run(task):
    arm,dv,seed=task; exe=ARMS[arm]
    dn,de=DIRS[dv]; norm=math.hypot(dn,de) or 1.0
    ln,le=10*dn/norm,10*de/norm
    alt=30.0; hold=alt+11.0; start=alt+13.0; end=start+60.0; seconds=end+30.0
    tag=f"{arm}_{dv}_s{seed}"
    cmd=[exe,"--real-nav","--profile","mag_clean","--pos-hold","--pos-hold-at",f"{hold:g}",
         "--pos-leg",f"{ln:.3f}:{le:.3f}","--gnss-outage",f"{start:g}:{end:g}",
         "--alt","30","--seed",str(seed),"--seconds",f"{seconds:g}",
         "--wind-window",f"{start:g}:{end:g}","--wind-force","1.0"]
    p=subprocess.run(cmd,capture_output=True,text=True,timeout=320)
    m=JSON_RE.findall(p.stdout+p.stderr)
    if not m: return dict(tag=tag,arm=arm,dir=dv,seed=seed,no_json=True)
    d=json.loads(m[-1].replace("nan","NaN"))
    o=dict(tag=tag,arm=arm,dir=dv,seed=seed,alt=30.0,outage_s=60,alt_at_outage=d.get("alt_at_outage"))
    for k in ("land_t","land_hold","land_pos_err","land_roll","max_roll_preland",
              "land_vz_down","max_desc_vz_down","final_alt","pos_rms","pos_max"):
        o[k]=d.get(k)
    o["landed"]=d.get("final_alt") is not None and d["final_alt"]<1.0
    return o
def main():
    os.makedirs(OUT,exist_ok=True)
    tasks=[(a,dv,s) for a in ARMS for dv in DIRS for s in range(1,11)]
    print("tasks",len(tasks),flush=True)
    with open(os.path.join(OUT,"results.jsonl"),"w",encoding="utf-8") as f, cf.ProcessPoolExecutor(max_workers=6) as ex:
        done=0
        for r in ex.map(run,tasks,chunksize=2):
            f.write(json.dumps(r)+"\n"); f.flush(); done+=1
            if done%50==0: print("done",done,flush=True)
    print("wrote",OUT,flush=True)
if __name__=="__main__": main()
