param([string]$TemplatePath, [string]$ReportPath)
$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0
function Clean($r) { (($r.Text -replace '[\r\a]', ' ' -replace '\s+', ' ').Trim()) }
try {
    $template = $word.Documents.Open((Resolve-Path -LiteralPath $TemplatePath).Path, $false, $true)
    $report = $word.Documents.Open((Resolve-Path -LiteralPath $ReportPath).Path, $false, $true)
    foreach ($sourceIndex in @(73, 75)) {
        $p = $template.Paragraphs.Item($sourceIndex)
        [ordered]@{
            Kind = "template-$sourceIndex"
            Text = Clean $p.Range
            Outline = $p.OutlineLevel
            FirstLineIndent = $p.Format.FirstLineIndent
            CharacterUnitFirstLineIndent = $p.Format.CharacterUnitFirstLineIndent
            LeftIndent = $p.Format.LeftIndent
            CharacterUnitLeftIndent = $p.Format.CharacterUnitLeftIndent
            Font = $p.Range.Characters.Item(1).Font.NameFarEast
            Size = $p.Range.Characters.Item(1).Font.Size
        } | ConvertTo-Json -Compress
    }
    $started = $false
    for ($i = 1; $i -le $report.Paragraphs.Count; $i++) {
        $p = $report.Paragraphs.Item($i)
        $text = Clean $p.Range
        if ($text -eq '第一章 绪论' -and $p.OutlineLevel -eq 1) { $started = $true }
        if ($started -and $text) {
            [ordered]@{
                Index = $i
                Text = $text
                Outline = $p.OutlineLevel
                FirstLineIndent = $p.Format.FirstLineIndent
                CharacterUnitFirstLineIndent = $p.Format.CharacterUnitFirstLineIndent
                LeftIndent = $p.Format.LeftIndent
                CharacterUnitLeftIndent = $p.Format.CharacterUnitLeftIndent
                Font = $p.Range.Characters.Item(1).Font.NameFarEast
                Size = $p.Range.Characters.Item(1).Font.Size
            } | ConvertTo-Json -Compress
            if ($i -gt 140) { break }
        }
    }
    $report.Close(0)
    $template.Close(0)
} finally {
    $word.Quit()
    [System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($word) | Out-Null
}
