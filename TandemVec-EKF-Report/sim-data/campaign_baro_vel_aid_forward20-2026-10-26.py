"""Forward-flight 20 s GNSS outage matrix, same-binary aid off/on A/B."""
import concurrent.futures as cf, json, math, os, re, subprocess
TMP=r"C:\Users\12631\AppData\Local\Temp"; EXE=TMP+r"\camp_aidonly\fw_real.exe"
OUT=TMP+r"\campaign_aid_fwd20"
DIRS={"N":(1,0),"E":(0,1),"S":(-1,0),"W":(0,-1),"NE":(1,1)}
DISTS=[3.0,10.0,25.0,50.0]
CONDS=["outage","both"]
SEEDS=range(1,11); SECONDS=45.0; STEP=20.0; FAULT="20:40"
JSON_RE=re.compile(r"SIL_REAL_JSON (\{.*\})")
TRUTH_RE=re.compile(r"^t=\s*([0-9.]+).*?truth=\(\s*(-?[0-9.]+),\s*(-?[0-9.]+)\)")
def one(t):
    cond,dv,dist,seed,aid=t; dn,de=DIRS[dv]; n=math.hypot(dn,de)
    ln,le=dist*dn/n,dist*de/n
    tag=f"{cond}_{dv}_{int(dist)}_s{seed}_a{aid}"
    cmd=[EXE,"--real-nav","--profile","mag_clean","--pos-hold","--pos-hold-at",str(STEP),
         "--pos-leg",f"{ln:.3f}:{le:.3f}","--flow","--seed",str(seed),"--seconds",str(SECONDS),
         "--gnss-outage",FAULT,"--baro-vel-aid",str(aid)]
    if cond=="both": cmd += ["--wind-window",FAULT,"--wind-force","1.0"]
    p=subprocess.run(cmd,capture_output=True,text=True,timeout=180)
    text=p.stdout+p.stderr
    mm=JSON_RE.findall(text)
    if not mm: return dict(tag=tag,cond=cond,dir=dv,dist=dist,seed=seed,aid=aid,no_json=True)
    d=json.loads(mm[-1].replace("nan","NaN"))
    truth=[]
    for line in text.splitlines():
        m=TRUTH_RE.match(line)
        if m: truth.append((float(m.group(1)),float(m.group(2)),float(m.group(3))))
    end_err=math.hypot(truth[-1][1]-ln,truth[-1][2]-le) if truth else None
    o=dict(tag=tag,cond=cond,dir=dv,dist=dist,seed=seed,aid=aid,leg_n=ln,leg_e=le,end_target_err=end_err)
    for k in ("pos_rms","pos_max","max_roll","max_pitch","final_alt","heading_aiding",
              "pos_degraded","outlier_rejects","reacquire_resets","baro_vel_attempts",
              "baro_vel_fused","baro_vel_rejected"):
        o[k]=d.get(k)
    o["nav_fail"]=bool((d.get("pos_rms") or 0)>3 or (d.get("pos_max") or 0)>10
                       or abs((d.get("final_alt") or 0)-10)>3 or (d.get("max_roll") or 0)>45)
    o["stab_fail"]=bool(abs((d.get("final_alt") or 0)-10)>3 or (d.get("max_roll") or 0)>45)
    return o
def main():
    os.makedirs(OUT,exist_ok=True)
    tasks=[(c,dv,dist,s,aid) for c in CONDS for dv in DIRS
           for dist in ([0.0] if dv=="N" else [])+DISTS for s in SEEDS for aid in (0,1)]
    print("tasks",len(tasks),flush=True)
    with open(os.path.join(OUT,"results.jsonl"),"w",encoding="utf-8") as f, cf.ProcessPoolExecutor(max_workers=6) as ex:
        done=0
        for r in ex.map(one,tasks,chunksize=2):
            f.write(json.dumps(r)+"\n"); f.flush(); done+=1
            if done%100==0: print("done",done,flush=True)
    print("wrote",OUT,flush=True)
if __name__=="__main__": main()
