param(
    [Parameter(Mandatory = $true)][string]$TemplatePath,
    [Parameter(Mandatory = $true)][string]$ReportPath,
    [string]$ProjectTitle = "慧语灵臂——基于Intel Core Ultra的具身智能机械臂系统"
)

$ErrorActionPreference = "Stop"
$errors = New-Object System.Collections.Generic.List[string]

function Clean-Text($range) {
    return (($range.Text -replace '[\r\a]', ' ' -replace '\s+', ' ').Trim())
}

function Nearly-Equal([double]$a, [double]$b, [double]$tolerance = 0.05) {
    return [Math]::Abs($a - $b) -le $tolerance
}

function Find-ParagraphIndex($document, [string]$text, [bool]$startsWith = $false, [int]$afterIndex = 0, [int]$outlineLevel = -1) {
    for ($i = [Math]::Max(1, $afterIndex + 1); $i -le $document.Paragraphs.Count; $i++) {
        $p = $document.Paragraphs.Item($i)
        $value = Clean-Text $p.Range
        $textMatches = ($startsWith -and $value.StartsWith($text)) -or (-not $startsWith -and $value -eq $text)
        $outlineMatches = ($outlineLevel -lt 0) -or ($p.OutlineLevel -eq $outlineLevel)
        if ($textMatches -and $outlineMatches) {
            return $i
        }
    }
    return 0
}

function Compare-Paragraph($label, $source, $target) {
    if ($null -eq $target) {
        $errors.Add("缺少段落：$label")
        return
    }
    $sourceFont = $source.Range.Characters.Item(1).Font
    $targetFont = $target.Range.Characters.Item(1).Font
    $fontProperties = @('Name', 'NameFarEast', 'Size', 'Bold', 'Italic', 'Underline')
    foreach ($property in $fontProperties) {
        if ($label -eq '原创性声明签名' -and $property -eq 'Name') { continue }
        $a = $sourceFont.$property
        $b = $targetFont.$property
        if ($property -eq 'Size') {
            if (-not (Nearly-Equal ([double]$a) ([double]$b))) { $errors.Add("$label 字体.$property：模板=$a，报告=$b") }
        }
        elseif ($a -ne $b) { $errors.Add("$label 字体.$property：模板=$a，报告=$b") }
    }

    $paragraphProperties = @(
        'Alignment', 'LeftIndent', 'RightIndent', 'FirstLineIndent',
        'LineSpacingRule', 'LineSpacing', 'SpaceBefore', 'SpaceAfter',
        'KeepWithNext', 'KeepTogether', 'WidowControl', 'PageBreakBefore'
    )
    foreach ($property in $paragraphProperties) {
        $a = $source.Format.$property
        $b = $target.Format.$property
        if ($property -in @('LeftIndent', 'RightIndent', 'FirstLineIndent', 'LineSpacing', 'SpaceBefore', 'SpaceAfter')) {
            if (-not (Nearly-Equal ([double]$a) ([double]$b))) { $errors.Add("$label 段落.$property：模板=$a，报告=$b") }
        }
        elseif ($a -ne $b) { $errors.Add("$label 段落.$property：模板=$a，报告=$b") }
    }
}

function Compare-PageSetup($label, $sourceSection, $targetSection) {
    $properties = @(
        'PageWidth', 'PageHeight', 'Orientation', 'TopMargin', 'BottomMargin',
        'LeftMargin', 'RightMargin', 'Gutter', 'HeaderDistance', 'FooterDistance',
        'VerticalAlignment', 'SectionStart', 'LayoutMode', 'LinesPage', 'CharsLine'
    )
    foreach ($property in $properties) {
        $a = $sourceSection.PageSetup.$property
        $b = $targetSection.PageSetup.$property
        if ($property -in @('PageWidth', 'PageHeight', 'TopMargin', 'BottomMargin', 'LeftMargin', 'RightMargin', 'Gutter', 'HeaderDistance', 'FooterDistance')) {
            if (-not (Nearly-Equal ([double]$a) ([double]$b))) { $errors.Add("$label 页面.$property：模板=$a，报告=$b") }
        }
        elseif ($a -ne $b) { $errors.Add("$label 页面.$property：模板=$a，报告=$b") }
    }
}

$templateFull = (Resolve-Path -LiteralPath $TemplatePath).Path
$reportFull = (Resolve-Path -LiteralPath $ReportPath).Path
$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0
$template = $null
$report = $null

try {
    $template = $word.Documents.Open($templateFull, $false, $true)
    $report = $word.Documents.Open($reportFull, $false, $true)
    $report.Repaginate()

    if ($report.CompatibilityMode -ne $template.CompatibilityMode) {
        $errors.Add("兼容模式：模板=$($template.CompatibilityMode)，报告=$($report.CompatibilityMode)")
    }
    if ($report.Sections.Count -ne 2) { $errors.Add("节数量应为2，实际为$($report.Sections.Count)") }
    Compare-PageSetup '封面节' $template.Sections.Item(1) $report.Sections.Item(1)
    Compare-PageSetup '正文节' $template.Sections.Item(2) $report.Sections.Item(2)

    $checks = @(
        @('封面竞赛中文', 2, '2026年（第十三届）英特尔杯大学生电子设计竞赛嵌入式AI专题赛', $false, 0),
        @('封面竞赛英文1', 3, '2026 Intel Cup Undergraduate Electronic Design Contest', $false, 0),
        @('封面竞赛英文2', 4, '- Embedded System Design Invitational Contest', $false, 0),
        @('封面作品设计报告', 5, '作品设计报告', $false, 0),
        @('封面Final Report', 6, 'Final Report', $false, 0),
        @('封面报告题目', 12, '报告题目：', $true, 0),
        @('封面学生姓名', 15, '学生姓名：', $true, 0),
        @('原创性声明竞赛名1', 21, '2026年（第十三届）英特尔杯大学生电子设计竞赛', $false, 0),
        @('原创性声明竞赛名2', 22, '嵌入式AI专题赛', $false, 0),
        @('原创性声明标题', 24, '参赛作品原创性声明', $false, 0),
        @('原创性声明签名', 29, '参赛队员签名：', $true, 0),
        @('原创性声明日期', 33, '日期：', $true, 0),
        @('中文标题', 36, $ProjectTitle, $false, 0),
        @('中文摘要标题', 38, '摘要', $false, 0),
        @('中文关键词', 42, '关键词：', $true, 0),
        @('英文标题', 44, 'HUIYU LINGBI: AN EMBODIED INTELLIGENT ROBOTIC ARM SYSTEM BASED ON INTEL CORE ULTRA', $false, 0),
        @('英文摘要标题', 48, 'ABSTRACT', $false, 0),
        @('英文关键词', 52, 'Keywords:', $true, 0),
        @('目录标题', 54, '目 录', $false, 0)
    )

    foreach ($check in $checks) {
        $targetIndex = Find-ParagraphIndex $report ([string]$check[2]) ([bool]$check[3]) ([int]$check[4])
        $target = if ($targetIndex -gt 0) { $report.Paragraphs.Item($targetIndex) } else { $null }
        Compare-Paragraph ([string]$check[0]) $template.Paragraphs.Item([int]$check[1]) $target
    }

    $declarationTitleIndex = Find-ParagraphIndex $report '参赛作品原创性声明'
    if ($declarationTitleIndex -gt 0) {
        $declarationBodyIndex = Find-ParagraphIndex $report '本团队郑重声明' $true $declarationTitleIndex
        $declarationBody = if ($declarationBodyIndex -gt 0) { $report.Paragraphs.Item($declarationBodyIndex) } else { $null }
        Compare-Paragraph '原创性声明正文' $template.Paragraphs.Item(26) $declarationBody
    }
    $abstractTitleIndex = Find-ParagraphIndex $report '摘要'
    if ($abstractTitleIndex -gt 0) {
        $abstractBody = $null
        for ($i = $abstractTitleIndex + 1; $i -le $report.Paragraphs.Count; $i++) {
            $candidate = $report.Paragraphs.Item($i)
            if (Clean-Text $candidate.Range) { $abstractBody = $candidate; break }
        }
        Compare-Paragraph '中文摘要正文' $template.Paragraphs.Item(40) $abstractBody
    }
    $englishAbstractTitleIndex = Find-ParagraphIndex $report 'ABSTRACT'
    if ($englishAbstractTitleIndex -gt 0) {
        $englishAbstractBody = $null
        for ($i = $englishAbstractTitleIndex + 1; $i -le $report.Paragraphs.Count; $i++) {
            $candidate = $report.Paragraphs.Item($i)
            if (Clean-Text $candidate.Range) { $englishAbstractBody = $candidate; break }
        }
        Compare-Paragraph '英文摘要正文' $template.Paragraphs.Item(50) $englishAbstractBody
    }
    $heading1Index = Find-ParagraphIndex $report '第一章 绪论' $false 0 1
    $heading1 = if ($heading1Index -gt 0) { $report.Paragraphs.Item($heading1Index) } else { $null }
    Compare-Paragraph '正文一级标题' $template.Paragraphs.Item(71) $heading1
    $heading2Index = Find-ParagraphIndex $report '1.1 项目背景' $false $heading1Index 3
    $heading2 = if ($heading2Index -gt 0) { $report.Paragraphs.Item($heading2Index) } else { $null }
    Compare-Paragraph '正文二级标题' $template.Paragraphs.Item(75) $heading2
    if ($heading1Index -gt 0) {
        $body = $null
        for ($i = $heading1Index + 1; $i -le $report.Paragraphs.Count; $i++) {
            $candidate = $report.Paragraphs.Item($i)
            if ((Clean-Text $candidate.Range) -and $candidate.OutlineLevel -eq 10 -and -not $candidate.Range.Information(12)) {
                $body = $candidate
                break
            }
        }
        Compare-Paragraph '正文普通段落' $template.Paragraphs.Item(73) $body
    }

    for ($si = 1; $si -le 2; $si++) {
        $header = $report.Sections.Item($si).Headers.Item(1)
        $headerText = Clean-Text $header.Range
        if (-not $headerText.Contains($ProjectTitle)) { $errors.Add("第${si}节页眉缺少当前项目标题") }
        if ($header.InlineShapes.Count -ne $template.Sections.Item($si).Headers.Item(1).InlineShapes.Count) {
            $errors.Add("第${si}节页眉图片数量与模板不一致")
        }
    }

    $footer = $report.Sections.Item(2).Footers.Item(1)
    $fieldCodes = @()
    foreach ($field in $footer.Range.Fields) { $fieldCodes += (Clean-Text $field.Code) }
    if (-not ($fieldCodes -contains 'PAGE')) { $errors.Add('正文页脚缺少PAGE域') }
    if (-not ($fieldCodes | Where-Object { $_ -like 'SECTIONPAGES*' })) { $errors.Add('正文页脚缺少SECTIONPAGES域') }
    if (-not (Clean-Text $footer.Range).StartsWith('第')) { $errors.Add('正文页脚文本格式异常') }

    if ($report.InlineShapes.Count -ne 1) { $errors.Add("正文内嵌图片应仅含模板封面标志1个，实际为$($report.InlineShapes.Count)") }
    if ($report.Shapes.Count -ne 0) { $errors.Add("正文不应含模板批注浮动形状，实际为$($report.Shapes.Count)") }
    if ($report.TablesOfContents.Count -ne 1) { $errors.Add("自动目录数量应为1，实际为$($report.TablesOfContents.Count)") }

    $placeholderTables = 0
    $dataTables = 0
    for ($ti = 1; $ti -le $report.Tables.Count; $ti++) {
        $table = $report.Tables.Item($ti)
        $tableText = Clean-Text $table.Range
        if ($tableText.Contains('【图片位置预留】')) {
            $placeholderTables++
            continue
        }
        $dataTables++
        if ($table.AllowAutoFit -ne $template.Tables.Item(1).AllowAutoFit) { $errors.Add("数据表$ti 自动调整属性不符") }
        foreach ($property in @('TopPadding', 'BottomPadding', 'LeftPadding', 'RightPadding')) {
            $a = $template.Tables.Item(1).$property
            $b = $table.$property
            if (-not (Nearly-Equal ([double]$a) ([double]$b))) { $errors.Add("数据表$ti $property：模板=$a，报告=$b") }
        }
        foreach ($borderId in @(-1, -2, -3, -4, -5, -6)) {
            if ($borderId -eq -5 -and $table.Rows.Count -lt 2) { continue }
            if ($borderId -eq -6 -and $table.Columns.Count -lt 2) { continue }
            $sourceBorder = $template.Tables.Item(1).Borders.Item($borderId)
            $targetBorder = $table.Borders.Item($borderId)
            if ($sourceBorder.LineStyle -ne $targetBorder.LineStyle -or $sourceBorder.LineWidth -ne $targetBorder.LineWidth) {
                $errors.Add("数据表$ti 边框$borderId 与模板不符")
            }
        }
    }

    $pages = $report.ComputeStatistics(2)
    $summary = [ordered]@{
        Pages = $pages
        Paragraphs = $report.Paragraphs.Count
        Sections = $report.Sections.Count
        Tables = $report.Tables.Count
        DataTables = $dataTables
        ImagePlaceholderTables = $placeholderTables
        InlineShapes = $report.InlineShapes.Count
        FloatingShapes = $report.Shapes.Count
        TOCs = $report.TablesOfContents.Count
        FooterFields = ($fieldCodes -join ',')
        Errors = $errors.Count
    }
    $summary | ConvertTo-Json -Compress
    foreach ($error in $errors) { Write-Output "ERROR: $error" }
    if ($errors.Count -gt 0) { throw "严格模板格式审计失败，共$($errors.Count)项。" }

    $report.Close(0)
    $report = $null
    $template.Close(0)
    $template = $null
}
finally {
    if ($null -ne $report) { $report.Close(0) }
    if ($null -ne $template) { $template.Close(0) }
    $word.Quit()
    [System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($word) | Out-Null
}
