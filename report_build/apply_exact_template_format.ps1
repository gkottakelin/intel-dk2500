param(
    [Parameter(Mandatory = $true)][string]$TemplatePath,
    [Parameter(Mandatory = $true)][string]$ReportPath,
    [string]$ProjectTitle = "慧语灵臂——基于Intel Core Ultra的具身智能机械臂系统"
)

$ErrorActionPreference = "Stop"

function Clean-Text($range) {
    return (($range.Text -replace '[\r\a]', ' ' -replace '\s+', ' ').Trim())
}

function Copy-ParagraphFormat($sourceParagraph, $targetParagraph, [int]$outlineLevel = 0) {
    if ($outlineLevel -gt 0) {
        # Setting OutlineLevel can apply a built-in heading style and overwrite
        # direct formatting, so it must happen before copying the template.
        $targetParagraph.OutlineLevel = $outlineLevel
    }
    $sourceFont = $sourceParagraph.Range.Characters.Item(1).Font.Duplicate
    $sourceFormat = $sourceParagraph.Format.Duplicate
    $targetParagraph.Range.Font = $sourceFont
    $targetParagraph.Format = $sourceFormat

    # Word does not consistently preserve every paragraph property when a
    # ParagraphFormat object is assigned across DOC/DOCX compatibility modes.
    # Reapply all measurable template properties explicitly.
    foreach ($property in @(
        'Alignment', 'LeftIndent', 'RightIndent', 'FirstLineIndent',
        'LineSpacingRule', 'LineSpacing', 'SpaceBefore', 'SpaceAfter',
        'SpaceBeforeAuto', 'SpaceAfterAuto', 'KeepWithNext', 'KeepTogether',
        'WidowControl', 'PageBreakBefore', 'DisableLineHeightGrid'
    )) {
        try { $targetParagraph.Format.$property = $sourceParagraph.Format.$property } catch {}
    }
}

function Copy-PageSetup($sourceSection, $targetSection) {
    $s = $sourceSection.PageSetup
    $t = $targetSection.PageSetup
    $t.PageWidth = $s.PageWidth
    $t.PageHeight = $s.PageHeight
    $t.Orientation = $s.Orientation
    $t.TopMargin = $s.TopMargin
    $t.BottomMargin = $s.BottomMargin
    $t.LeftMargin = $s.LeftMargin
    $t.RightMargin = $s.RightMargin
    $t.Gutter = $s.Gutter
    $t.HeaderDistance = $s.HeaderDistance
    $t.FooterDistance = $s.FooterDistance
    $t.VerticalAlignment = $s.VerticalAlignment
    $t.SectionStart = $s.SectionStart
    $t.LinesPage = $s.LinesPage
    $t.CharsLine = $s.CharsLine
    # Setting line/character counts changes LayoutMode in Word; apply the
    # template mode last so both line and character grids remain enabled.
    $t.LayoutMode = $s.LayoutMode
    $t.DifferentFirstPageHeaderFooter = 0
}

function Replace-RangeTextPreserveFormat($range, [string]$oldText, [string]$newText) {
    $search = $range.Duplicate
    $search.Find.ClearFormatting()
    $search.Find.Text = $oldText
    if ($search.Find.Execute()) {
        $font = $search.Font.Duplicate
        $search.Text = $newText
        $search.Font = $font
        return $true
    }
    return $false
}

$templateFull = (Resolve-Path -LiteralPath $TemplatePath).Path
$reportFull = (Resolve-Path -LiteralPath $ReportPath).Path
$oldTemplateTitle = "二甲醚清洁燃料均质压燃燃烧数值模拟研究"

$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0
$template = $null
$report = $null
$stage = '初始化'

try {
    $stage = '打开模板和报告'
    $template = $word.Documents.Open($templateFull, $false, $true)
    $report = $word.Documents.Open($reportFull)
    $report.SetCompatibilityMode($template.CompatibilityMode)

    # Copy the original competition logo from the template body.
    $stage = '复制封面标志'
    $logoRange = $report.Content.Duplicate
    $logoRange.Find.Text = '[[TEMPLATE_LOGO]]'
    if ($logoRange.Find.Execute()) {
        $template.InlineShapes.Item(1).Range.Copy()
        $logoRange.Text = ''
        $logoRange.Collapse(1)
        $logoRange.Paste()
        $logoRange.ParagraphFormat.Alignment = 1
    }

    # Generate TOC before formatting it.
    $stage = '更新目录字段'
    $report.Fields.Update() | Out-Null
    foreach ($toc in $report.TablesOfContents) {
        $toc.Update()
    }

    # Exact A4 page setup from template sections 1 and 2.
    $stage = '复制页面设置'
    Copy-PageSetup $template.Sections.Item(1) $report.Sections.Item(1)
    Copy-PageSetup $template.Sections.Item(2) $report.Sections.Item(2)
    $report.Sections.Item(1).Headers.Item(1).LinkToPrevious = $false
    $report.Sections.Item(1).Footers.Item(1).LinkToPrevious = $false
    $report.Sections.Item(2).Headers.Item(1).LinkToPrevious = $false
    $report.Sections.Item(2).Footers.Item(1).LinkToPrevious = $false

    # Copy header/footer objects, including the template's header image.
    $stage = '复制页眉页脚'
    $report.Sections.Item(1).Headers.Item(1).Range.FormattedText = $template.Sections.Item(1).Headers.Item(1).Range.FormattedText
    $report.Sections.Item(1).Footers.Item(1).Range.FormattedText = $template.Sections.Item(1).Footers.Item(1).Range.FormattedText
    $report.Sections.Item(2).Headers.Item(1).Range.FormattedText = $template.Sections.Item(2).Headers.Item(1).Range.FormattedText
    $report.Sections.Item(2).Footers.Item(1).Range.FormattedText = $template.Sections.Item(2).Footers.Item(1).Range.FormattedText
    [void](Replace-RangeTextPreserveFormat $report.Sections.Item(1).Headers.Item(1).Range $oldTemplateTitle $ProjectTitle)
    [void](Replace-RangeTextPreserveFormat $report.Sections.Item(2).Headers.Item(1).Range $oldTemplateTitle $ProjectTitle)

    # Replace the sample's fixed total-page number with a live section-page field.
    $stage = '设置正文页码域'
    $footerRange = $report.Sections.Item(2).Footers.Item(1).Range.Duplicate
    $footerRange.Find.Text = '6'
    if ($footerRange.Find.Execute()) {
        $footerRange.Text = ''
        $footerRange.Collapse(1)
        [void]$report.Sections.Item(2).Footers.Item(1).Range.Fields.Add($footerRange, -1, 'SECTIONPAGES', $true)
    }
    $report.Sections.Item(2).Footers.Item(1).PageNumbers.RestartNumberingAtSection = $true
    $report.Sections.Item(2).Footers.Item(1).PageNumbers.StartingNumber = 1

    $sample = @{
        ContestCN = $template.Paragraphs.Item(2)
        ContestEN1 = $template.Paragraphs.Item(3)
        ContestEN2 = $template.Paragraphs.Item(4)
        ReportTitle = $template.Paragraphs.Item(5)
        FinalReport = $template.Paragraphs.Item(6)
        CoverTopic = $template.Paragraphs.Item(12)
        Student = $template.Paragraphs.Item(15)
        Teacher = $template.Paragraphs.Item(16)
        School = $template.Paragraphs.Item(17)
        DeclarationContest1 = $template.Paragraphs.Item(21)
        DeclarationContest2 = $template.Paragraphs.Item(22)
        DeclarationTitle = $template.Paragraphs.Item(24)
        DeclarationBody = $template.Paragraphs.Item(26)
        Signature = $template.Paragraphs.Item(29)
        Date = $template.Paragraphs.Item(33)
        ChineseTitle = $template.Paragraphs.Item(36)
        ChineseAbstractTitle = $template.Paragraphs.Item(38)
        ChineseAbstractBody = $template.Paragraphs.Item(40)
        ChineseKeywords = $template.Paragraphs.Item(42)
        EnglishTitle = $template.Paragraphs.Item(44)
        EnglishAbstractTitle = $template.Paragraphs.Item(48)
        EnglishAbstractBody = $template.Paragraphs.Item(50)
        EnglishKeywords = $template.Paragraphs.Item(52)
        TocTitle = $template.Paragraphs.Item(54)
        Toc1 = $template.Paragraphs.Item(56)
        Toc2 = $template.Paragraphs.Item(62)
        Toc3 = $template.Paragraphs.Item(63)
        Heading1 = $template.Paragraphs.Item(71)
        Body = $template.Paragraphs.Item(73)
        Heading2 = $template.Paragraphs.Item(75)
        Heading3 = $template.Paragraphs.Item(77)
        ListItem = $template.Paragraphs.Item(79)
        TableCaption = $template.Paragraphs.Item(114)
        FigureCaption = $template.Paragraphs.Item(164)
        Reference = $template.Paragraphs.Item(184)
    }

    $inDeclaration = $false
    $inChineseAbstract = $false
    $inEnglishAbstract = $false
    $inToc = $false
    $inBody = $false

    for ($i = 1; $i -le $report.Paragraphs.Count; $i++) {
        $stage = "格式化段落 $i/$($report.Paragraphs.Count)"
        $p = $report.Paragraphs.Item($i)
        $text = Clean-Text $p.Range
        if (-not $text) { continue }

        if ($text -eq '2026年（第十三届）英特尔杯大学生电子设计竞赛嵌入式AI专题赛') { Copy-ParagraphFormat $sample.ContestCN $p; continue }
        if ($text -eq '2026 Intel Cup Undergraduate Electronic Design Contest') { Copy-ParagraphFormat $sample.ContestEN1 $p; continue }
        if ($text -eq '- Embedded System Design Invitational Contest') { Copy-ParagraphFormat $sample.ContestEN2 $p; continue }
        if ($text -eq '作品设计报告') { Copy-ParagraphFormat $sample.ReportTitle $p; continue }
        if ($text -eq 'Final Report') { Copy-ParagraphFormat $sample.FinalReport $p; continue }
        if ($text.StartsWith('报告题目：')) { Copy-ParagraphFormat $sample.CoverTopic $p; continue }
        if ($text.StartsWith('学生姓名：')) { Copy-ParagraphFormat $sample.Student $p; continue }
        if ($text.StartsWith('指导教师：')) { Copy-ParagraphFormat $sample.Teacher $p; continue }
        if ($text.StartsWith('参赛学校：')) { Copy-ParagraphFormat $sample.School $p; continue }
        if ($text -eq '2026年（第十三届）英特尔杯大学生电子设计竞赛') { Copy-ParagraphFormat $sample.DeclarationContest1 $p; continue }
        if ($text -eq '嵌入式AI专题赛') { Copy-ParagraphFormat $sample.DeclarationContest2 $p; continue }
        if ($text -eq '参赛作品原创性声明') { Copy-ParagraphFormat $sample.DeclarationTitle $p; $inDeclaration = $true; continue }
        if ($text.StartsWith('参赛队员签名：')) { $inDeclaration = $false; Copy-ParagraphFormat $sample.Signature $p; continue }
        if ($text.StartsWith('指导教师签名：')) { Copy-ParagraphFormat $sample.Signature $p; continue }
        if ($text.StartsWith('日期：')) { Copy-ParagraphFormat $sample.Date $p; continue }
        if ($inDeclaration) { Copy-ParagraphFormat $sample.DeclarationBody $p; continue }

        if ($text -eq $ProjectTitle) { Copy-ParagraphFormat $sample.ChineseTitle $p; continue }
        if ($text -eq '摘要') { Copy-ParagraphFormat $sample.ChineseAbstractTitle $p; $inChineseAbstract = $true; continue }
        if ($text.StartsWith('关键词：')) { $inChineseAbstract = $false; Copy-ParagraphFormat $sample.ChineseKeywords $p; continue }
        if ($inChineseAbstract) { Copy-ParagraphFormat $sample.ChineseAbstractBody $p; continue }
        if ($text -eq 'HUIYU LINGBI: AN EMBODIED INTELLIGENT ROBOTIC ARM SYSTEM BASED ON INTEL CORE ULTRA') { Copy-ParagraphFormat $sample.EnglishTitle $p; continue }
        if ($text -eq 'ABSTRACT') { Copy-ParagraphFormat $sample.EnglishAbstractTitle $p; $inEnglishAbstract = $true; continue }
        if ($text.StartsWith('Keywords:')) { $inEnglishAbstract = $false; Copy-ParagraphFormat $sample.EnglishKeywords $p; continue }
        if ($inEnglishAbstract) { Copy-ParagraphFormat $sample.EnglishAbstractBody $p; continue }

        if ($text -eq '目 录') { Copy-ParagraphFormat $sample.TocTitle $p; $inToc = $true; continue }
        if ($text -eq '第一章 绪论') { $inToc = $false; $inBody = $true; Copy-ParagraphFormat $sample.Heading1 $p 1; continue }
        if ($inToc) {
            $styleName = ''
            try { $styleName = $p.Range.Style.NameLocal } catch {}
            if ($styleName -match '3$') { Copy-ParagraphFormat $sample.Toc3 $p }
            elseif ($styleName -match '2$') { Copy-ParagraphFormat $sample.Toc2 $p }
            else { Copy-ParagraphFormat $sample.Toc1 $p }
            continue
        }

        if (-not $inBody) { continue }
        if ($p.Range.Information(12)) { continue }

        $outline = $p.OutlineLevel
        if ($outline -eq 1) { Copy-ParagraphFormat $sample.Heading1 $p 1; continue }
        if ($outline -eq 2) { Copy-ParagraphFormat $sample.Heading2 $p 2; continue }
        if ($outline -eq 3) { Copy-ParagraphFormat $sample.Heading3 $p 3; continue }
        if ($text -match '^表([0-9A-Z]+)[-－]') { Copy-ParagraphFormat $sample.TableCaption $p; continue }
        if ($text -match '^图([0-9A-Z]+)[-－]') { Copy-ParagraphFormat $sample.FigureCaption $p; continue }
        if ($text -match '^\[\d+\]') { Copy-ParagraphFormat $sample.Reference $p; continue }

        $styleName = ''
        try { $styleName = $p.Range.Style.NameLocal } catch {}
        if ($styleName -match '列表|List') { Copy-ParagraphFormat $sample.ListItem $p; continue }
        if ($styleName -eq '模板代码' -or $styleName -eq '模板公式') { continue }
        Copy-ParagraphFormat $sample.Body $p
    }

    # Exact table font, paragraph layout, padding, autofit, and three-line borders.
    $templateCellParagraph = $template.Tables.Item(1).Cell(1, 1).Range.Paragraphs.Item(1)
    for ($ti = 1; $ti -le $report.Tables.Count; $ti++) {
        $stage = "格式化表格 $ti/$($report.Tables.Count)"
        $table = $report.Tables.Item($ti)
        $tableText = Clean-Text $table.Range
        if ($tableText.Contains('【图片位置预留】')) { continue }
        $table.AllowAutoFit = $template.Tables.Item(1).AllowAutoFit
        $table.TopPadding = $template.Tables.Item(1).TopPadding
        $table.BottomPadding = $template.Tables.Item(1).BottomPadding
        $table.LeftPadding = $template.Tables.Item(1).LeftPadding
        $table.RightPadding = $template.Tables.Item(1).RightPadding
        foreach ($borderId in @(-1, -2, -3, -4, -5, -6)) {
            if ($borderId -eq -5 -and $table.Rows.Count -lt 2) { continue }
            if ($borderId -eq -6 -and $table.Columns.Count -lt 2) { continue }
            $sourceBorder = $template.Tables.Item(1).Borders.Item($borderId)
            $targetBorder = $table.Borders.Item($borderId)
            $targetBorder.LineStyle = $sourceBorder.LineStyle
            if ($sourceBorder.LineStyle -ne 0) {
                $targetBorder.LineWidth = $sourceBorder.LineWidth
            }
            $targetBorder.Color = $sourceBorder.Color
        }
        # Apply the template cell typography and paragraph layout to the whole
        # table range at once. This also handles merged cells safely.
        $table.Range.Font = $templateCellParagraph.Range.Characters.Item(1).Font.Duplicate
        $table.Range.ParagraphFormat = $templateCellParagraph.Format.Duplicate
    }

    foreach ($toc in $report.TablesOfContents) {
        $stage = '更新目录页码'
        $toc.UpdatePageNumbers()
    }
    $stage = '更新全文字段'
    $report.Fields.Update() | Out-Null
    $stage = '重新分页'
    $report.Repaginate()
    $stage = '统计页数'
    $pages = $report.ComputeStatistics(2)
    $stage = '保存报告'
    $report.Save()
    Write-Output ('Exact template formatting applied. Pages={0} Paragraphs={1} Tables={2}' -f $pages, $report.Paragraphs.Count, $report.Tables.Count)

    $report.Close(0)
    $report = $null
    $template.Close(0)
    $template = $null
}
catch {
    Write-Output "FAILED_STAGE: $stage"
    Write-Output $_.Exception.Message
    Write-Output $_.ScriptStackTrace
    throw
}
finally {
    if ($null -ne $report) { $report.Close(0) }
    if ($null -ne $template) { $template.Close(0) }
    $word.Quit()
    [System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($word) | Out-Null
}
