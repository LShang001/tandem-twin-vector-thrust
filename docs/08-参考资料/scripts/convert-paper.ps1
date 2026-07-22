# convert-paper.ps1 — PDF → Markdown 辅助脚本
# 用法：
#   .\convert-paper.ps1 -PaperPath "papers/REF-xxx"
#   .\convert-paper.ps1 -PaperPath "papers/REF-xxx" -UseV1  # 轻量模式（免Token）
#   .\convert-paper.ps1 -PaperPath "papers/REF-xxx" -Preview  # 预览前3页

param(
    [Parameter(Mandatory=$true)]
    [string]$PaperPath,

    [switch]$UseV1,
    [switch]$Preview,
    [string]$Language = "en"
)

$MINERU_SCRIPTS = "$env:USERPROFILE\.agents\skills\mineru-pdf-markdown\scripts"

if (-not (Test-Path $PaperPath)) {
    Write-Error "目录不存在: $PaperPath"
    exit 1
}

$pdfPath = Join-Path $PaperPath "paper.pdf"
if (-not (Test-Path $pdfPath)) {
    Write-Error "未找到 paper.pdf: $pdfPath"
    exit 1
}

Write-Host "=== 文献转换 ===" -ForegroundColor Cyan
Write-Host "文献目录: $PaperPath"
Write-Host "PDF 文件 : $pdfPath"

# 验证 PDF 真伪
Write-Host "`n[1/3] 验证 PDF 真伪..." -ForegroundColor Yellow
try {
    $header = Get-Content -Path $pdfPath -Encoding Byte -TotalCount 5 -ErrorAction Stop
    $magic = [System.Text.Encoding]::ASCII.GetString($header[0..4])
    if ($magic -ne "%PDF-") {
        Write-Error "这不是有效的 PDF 文件！文件头: $magic"
        Write-Host "可能是期刊登录页 HTML，请用其它方式重新下载。" -ForegroundColor Red
        exit 1
    }
    Write-Host "  PDF 有效" -ForegroundColor Green
} catch {
    Write-Error "无法读取 PDF 文件: $_"
    exit 1
}

# 转换
Write-Host "`n[2/3] MinerU 转换..." -ForegroundColor Yellow

$outDir = $PaperPath  # 输出到文献目录本身

if ($UseV1) {
    # v1 Agent API（免 Token，轻量）
    $agentScript = Join-Path $MINERU_SCRIPTS "agent_parse.py"
    $args = @($agentScript, $pdfPath, "-o", $outDir, "--language", $Language)
    if ($Preview) {
        $args += "--pages", "1-3"
    }
    & py -3.12 @args
} else {
    # v4 精准 API
    $convertScript = Join-Path $MINERU_SCRIPTS "mineru_convert.py"
    $args = @($convertScript, $pdfPath, "-o", $outDir, "--language", $Language, "--model-version", "vlm")
    if ($Preview) {
        $args += "--preset", "preview"
    }
    & py -3.12 @args
}

if ($LASTEXITCODE -ne 0) {
    Write-Error "MinerU 转换失败，退出码: $LASTEXITCODE"
    exit $LASTEXITCODE
}

# 重命名 full.md → paper.md
$fullMd = Join-Path $outDir "full.md"
$paperMd = Join-Path $outDir "paper.md"
if (Test-Path $fullMd) {
    Move-Item -Force $fullMd $paperMd
    Write-Host "  full.md → paper.md" -ForegroundColor Green
}

# 质量审计
Write-Host "`n[3/3] 质量审计..." -ForegroundColor Yellow
$qualityScript = Join-Path $MINERU_SCRIPTS "mineru_quality.py"
& py -3.12 $qualityScript $outDir

Write-Host "`n=== 完成 ===" -ForegroundColor Cyan
Write-Host "输出文件:" -ForegroundColor White

Get-ChildItem -Path $outDir -File | ForEach-Object {
    $size = "{0,8:N0} KB" -f ($_.Length / 1KB)
    Write-Host "  $($_.Name)  $size"
}

Write-Host "`n下一步："
Write-Host "  1. 审阅 paper.md 内容质量"
Write-Host "  2. 撰写 notes.md 分析笔记"
Write-Host "  3. 更新 ../参考资料注册表.md"
