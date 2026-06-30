param(
    [Parameter(Mandatory = $true)][string]$TemplatePath,
    [Parameter(Mandatory = $true)][string]$ReportPath
)
$ErrorActionPreference = 'Stop'
$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0
$template = $null
$report = $null
try {
    $template = $word.Documents.Open((Resolve-Path -LiteralPath $TemplatePath).Path, $false, $true)
    $report = $word.Documents.Open((Resolve-Path -LiteralPath $ReportPath).Path)
    $report.SetCompatibilityMode($template.CompatibilityMode)
    for ($i = 1; $i -le 2; $i++) {
        $source = $template.Sections.Item($i).PageSetup
        $target = $report.Sections.Item($i).PageSetup
        $target.LinesPage = $source.LinesPage
        $target.CharsLine = $source.CharsLine
        $target.LayoutMode = $source.LayoutMode
    }
    foreach ($toc in $report.TablesOfContents) { $toc.UpdatePageNumbers() }
    $report.Fields.Update() | Out-Null
    $report.Repaginate()
    $pages = $report.ComputeStatistics(2)
    $report.Save()
    Write-Output "Template page grid applied. Pages=$pages"
    $report.Close(0)
    $report = $null
    $template.Close(0)
    $template = $null
} finally {
    if ($null -ne $report) { $report.Close(0) }
    if ($null -ne $template) { $template.Close(0) }
    $word.Quit()
    [System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($word) | Out-Null
}
