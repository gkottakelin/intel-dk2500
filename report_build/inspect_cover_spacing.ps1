param([string]$TemplatePath, [string]$ReportPath)
$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0
try {
    $template = $word.Documents.Open((Resolve-Path -LiteralPath $TemplatePath).Path, $false, $true)
    $report = $word.Documents.Open((Resolve-Path -LiteralPath $ReportPath).Path, $false, $true)
    $target = $null
    [ordered]@{ Kind = 'template-document'; CompatibilityMode = $template.CompatibilityMode } | ConvertTo-Json -Compress
    [ordered]@{ Kind = 'report-document'; CompatibilityMode = $report.CompatibilityMode } | ConvertTo-Json -Compress
    for ($i = 1; $i -le $report.Paragraphs.Count; $i++) {
        if (($report.Paragraphs.Item($i).Range.Text -replace '[\r\a]', '').Trim() -eq '作品设计报告') {
            $target = $report.Paragraphs.Item($i)
            break
        }
    }
    foreach ($entry in @(@('template', $template.Paragraphs.Item(5)), @('report', $target))) {
        $p = $entry[1]
        [ordered]@{
            Kind = $entry[0]
            Style = $p.Range.Style.NameLocal
            SpaceBefore = $p.Format.SpaceBefore
            SpaceAfter = $p.Format.SpaceAfter
            SpaceBeforeAuto = $p.Format.SpaceBeforeAuto
            SpaceAfterAuto = $p.Format.SpaceAfterAuto
            LineUnitBefore = $p.Format.LineUnitBefore
            LineUnitAfter = $p.Format.LineUnitAfter
            DisableLineHeightGrid = $p.Format.DisableLineHeightGrid
            LineSpacingRule = $p.Format.LineSpacingRule
            LineSpacing = $p.Format.LineSpacing
        } | ConvertTo-Json -Compress
    }
    foreach ($entry in @(@('template-section1', $template.Sections.Item(1)), @('report-section1', $report.Sections.Item(1)), @('template-section2', $template.Sections.Item(2)), @('report-section2', $report.Sections.Item(2)))) {
        $setup = $entry[1].PageSetup
        [ordered]@{
            Kind = $entry[0]
            LayoutMode = $setup.LayoutMode
            LinesPage = $setup.LinesPage
            CharsLine = $setup.CharsLine
        } | ConvertTo-Json -Compress
    }
    $report.Close(0)
    $template.Close(0)
} finally {
    $word.Quit()
    [System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($word) | Out-Null
}
