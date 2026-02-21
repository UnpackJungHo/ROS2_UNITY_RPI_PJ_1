using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
#endif

public static class TrainingAnalyticsPanelBuilder
{
    private const string CanvasName = "Canvas_1920x1080";
    private const string FallbackFontPath = "Assets/TextMesh Pro/Resources/Fonts & Materials/LiberationSans SDF.asset";

    private struct LineInstruction
    {
        public RectTransform Parent;
        public Vector2[] Points;
        public Color Color;
        public float Thickness;
        public bool Dashed;
    }

    private static readonly List<LineInstruction> QueuedLines = new List<LineInstruction>();

#if UNITY_EDITOR
    [MenuItem("Tools/GUI/Build Training Analytics Panel")]
    public static void BuildFromMenu()
    {
        Build();
    }
#endif

    public static void Build()
    {
        var canvasGo = GameObject.Find(CanvasName);
        if (canvasGo == null)
        {
            var canvas = Object.FindObjectOfType<Canvas>();
            if (canvas != null)
            {
                canvasGo = canvas.gameObject;
            }
        }

        if (canvasGo == null)
        {
            Debug.LogError("[TrainingAnalyticsPanelBuilder] Canvas not found.");
            return;
        }

        NormalizeCanvas(canvasGo);

        var old = canvasGo.transform.Find("TrainingAnalyticsUI");
        if (old != null)
        {
            Object.DestroyImmediate(old.gameObject);
        }

        DeactivateIfExists(canvasGo.transform, "DrivingTrainerUI");
        DeactivateIfExists(canvasGo.transform, "ScenarioSelectionUI");
        DeactivateIfExists(canvasGo.transform, "SimulationMonitorUI");

        var ctx = new UiContext
        {
            Font = LoadFont(),
            RoundSprite = LoadRoundSprite(),
            Blue900 = Hex("1F2F7A"),
            Blue800 = Hex("273B91"),
            Blue700 = Hex("324FA9"),
            Blue300 = Hex("A6B5E8"),
            Gray900 = Hex("2E3445"),
            Gray700 = Hex("6A7388"),
            Gray600 = Hex("8B94A8"),
            Gray500 = Hex("AAB1C1"),
            Gray400 = Hex("C2C8D5"),
            Gray300 = Hex("DDE2EC"),
            Gray200 = Hex("E8ECF4"),
            Gray150 = Hex("EEF2F8"),
            Gray100 = Hex("F5F7FB"),
            White = Color.white,
            Green = Hex("22C55E"),
            Yellow = Hex("EAB308"),
            Red = Hex("F87171")
        };

        var root = CreateRect("TrainingAnalyticsUI", canvasGo.transform);
        Stretch(root, 0f, 0f, 0f, 0f);

        BuildBackground(root, ctx);
        BuildTopBar(root, ctx);
        BuildBody(root, ctx);
        BuildBottomBar(root, ctx);

        Canvas.ForceUpdateCanvases();
        DrawQueuedLines();
        Canvas.ForceUpdateCanvases();

#if UNITY_EDITOR
        EditorUtility.SetDirty(canvasGo);
        EditorSceneManager.MarkSceneDirty(UnityEngine.SceneManagement.SceneManager.GetActiveScene());
#endif

        Debug.Log("[TrainingAnalyticsPanelBuilder] TrainingAnalyticsUI rebuilt.");
    }

    private static void NormalizeCanvas(GameObject canvasGo)
    {
        var canvas = canvasGo.GetComponent<Canvas>();
        if (canvas == null)
        {
            canvas = canvasGo.AddComponent<Canvas>();
        }

        var scaler = canvasGo.GetComponent<CanvasScaler>();
        if (scaler == null)
        {
            scaler = canvasGo.AddComponent<CanvasScaler>();
        }

        if (canvasGo.GetComponent<GraphicRaycaster>() == null)
        {
            canvasGo.AddComponent<GraphicRaycaster>();
        }

        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        canvas.worldCamera = null;

        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920f, 1080f);
        scaler.screenMatchMode = CanvasScaler.ScreenMatchMode.MatchWidthOrHeight;
        scaler.matchWidthOrHeight = 0.5f;
        scaler.referencePixelsPerUnit = 100f;

        var rt = canvasGo.GetComponent<RectTransform>();
        if (rt == null)
        {
            rt = canvasGo.AddComponent<RectTransform>();
        }

        rt.anchorMin = Vector2.zero;
        rt.anchorMax = Vector2.one;
        rt.pivot = new Vector2(0.5f, 0.5f);
        rt.offsetMin = Vector2.zero;
        rt.offsetMax = Vector2.zero;
        rt.localScale = Vector3.one;
        rt.localRotation = Quaternion.identity;
    }

    private static void DeactivateIfExists(Transform parent, string name)
    {
        var t = parent.Find(name);
        if (t != null)
        {
            t.gameObject.SetActive(false);
        }
    }

    private static void BuildBackground(RectTransform root, UiContext ctx)
    {
        var bg = CreateRect("Background", root);
        Stretch(bg, 0f, 0f, 0f, 0f);
        AddImage(bg, ctx.Gray100, ctx.RoundSprite);
    }

    private static void BuildTopBar(RectTransform root, UiContext ctx)
    {
        var top = CreateRect("TopBar", root);
        AnchorTop(top, 64f);
        AddImage(top, ctx.White, ctx.RoundSprite);

        var line = CreateRect("BottomLine", top);
        AnchorBottom(line, 1f);
        AddImage(line, ctx.Gray200, null);

        var content = CreateRect("Content", top);
        Stretch(content, 24f, 16f, 24f, 12f);

        var brand = CreateRect("Brand", content);
        SetAnchor(brand, new Vector2(0f, 0.5f), new Vector2(0f, 0.5f), new Vector2(0f, 0.5f));
        brand.sizeDelta = new Vector2(340f, 42f);
        brand.anchoredPosition = new Vector2(0f, -1f);

        var logo = CreateRect("Logo", brand);
        SetAnchor(logo, new Vector2(0f, 0.5f), new Vector2(0f, 0.5f), new Vector2(0f, 0.5f));
        logo.sizeDelta = new Vector2(28f, 28f);
        logo.anchoredPosition = new Vector2(14f, 0f);
        AddImage(logo, ctx.Blue800, ctx.RoundSprite);

        var logoGlyph = CreateText("Glyph", logo, "✦", ctx.Font, 16, ctx.White, FontStyles.Bold, TextAlignmentOptions.Center, false);
        Stretch(logoGlyph.rectTransform, 0f, 0f, 0f, 0f);

        var brandText = CreateText("BrandText", brand, "Autonomous <color=#A6B5E8>Driving</color>", ctx.Font, 24, ctx.Blue900, FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(brandText.rectTransform, new Vector2(0f, 0f), new Vector2(1f, 1f), new Vector2(0f, 0.5f));
        brandText.rectTransform.offsetMin = new Vector2(42f, 0f);
        brandText.rectTransform.offsetMax = new Vector2(0f, 0f);

        var nav = CreateRect("Nav", content);
        SetAnchor(nav, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f));
        nav.sizeDelta = new Vector2(440f, 30f);
        var navLayout = nav.gameObject.AddComponent<HorizontalLayoutGroup>();
        navLayout.spacing = 28f;
        navLayout.childAlignment = TextAnchor.MiddleCenter;
        navLayout.childControlHeight = false;
        navLayout.childControlWidth = false;
        navLayout.childForceExpandHeight = false;
        navLayout.childForceExpandWidth = false;

        CreateNavItem(nav, "Dashboard", ctx.Gray700, ctx);
        CreateNavItem(nav, "Analytics", ctx.Blue800, ctx, true);
        CreateNavItem(nav, "Simulation", ctx.Gray700, ctx);
        CreateNavItem(nav, "Settings", ctx.Gray700, ctx);

        var user = CreateRect("User", content);
        SetAnchor(user, new Vector2(1f, 0.5f), new Vector2(1f, 0.5f), new Vector2(1f, 0.5f));
        user.sizeDelta = new Vector2(270f, 44f);

        var info = CreateRect("Info", user);
        SetAnchor(info, new Vector2(0f, 0f), new Vector2(1f, 1f), new Vector2(1f, 0.5f));
        info.offsetMin = new Vector2(0f, 0f);
        info.offsetMax = new Vector2(-52f, 0f);

        var name = CreateText("Name", info, "Dr. Sarah Chen", ctx.Font, 20, ctx.Gray900, FontStyles.Bold, TextAlignmentOptions.Right, false);
        SetAnchor(name.rectTransform, new Vector2(0f, 0.52f), new Vector2(1f, 1f), new Vector2(1f, 1f));
        name.rectTransform.offsetMin = Vector2.zero;
        name.rectTransform.offsetMax = Vector2.zero;
        ConfigureSingleLineResponsive(name, 12f, 20f);

        var role = CreateText("Role", info, "Lead Engineer", ctx.Font, 14, ctx.Gray500, FontStyles.Bold, TextAlignmentOptions.Right, false);
        SetAnchor(role.rectTransform, new Vector2(0f, 0f), new Vector2(1f, 0.5f), new Vector2(1f, 0f));
        role.rectTransform.offsetMin = Vector2.zero;
        role.rectTransform.offsetMax = Vector2.zero;

        var avatar = CreateRect("Avatar", user);
        SetAnchor(avatar, new Vector2(1f, 0.5f), new Vector2(1f, 0.5f), new Vector2(1f, 0.5f));
        avatar.sizeDelta = new Vector2(40f, 40f);
        avatar.anchoredPosition = new Vector2(0f, 0f);
        AddImage(avatar, Hex("E5E9F4"), ctx.RoundSprite);
        var avatarGlyph = CreateText("Icon", avatar, "•", ctx.Font, 20, ctx.Gray700, FontStyles.Bold, TextAlignmentOptions.Center, false);
        Stretch(avatarGlyph.rectTransform, 0f, 0f, 0f, 0f);
    }

    private static void CreateNavItem(RectTransform parent, string text, Color color, UiContext ctx, bool active = false)
    {
        var item = CreateRect(text, parent);
        var le = item.gameObject.AddComponent<LayoutElement>();
        le.preferredWidth = active ? 84f : 76f;
        le.preferredHeight = 28f;

        var label = CreateText("Label", item, text, ctx.Font, 14, color, active ? FontStyles.Bold : FontStyles.Bold, TextAlignmentOptions.Center, false);
        Stretch(label.rectTransform, 0f, 0f, 0f, 0f);
    }

    private static void BuildBody(RectTransform root, UiContext ctx)
    {
        var body = CreateRect("Body", root);
        Stretch(body, 0f, 42f, 0f, 64f);

        var sidebar = CreateRect("LeftSidebar", body);
        SetAnchor(sidebar, new Vector2(0f, 0f), new Vector2(0f, 1f), new Vector2(0f, 0.5f));
        sidebar.sizeDelta = new Vector2(378f, 0f);
        AddImage(sidebar, ctx.Gray100, null);

        var divider = CreateRect("RightBorder", sidebar);
        SetAnchor(divider, new Vector2(1f, 0f), new Vector2(1f, 1f), new Vector2(1f, 0.5f));
        divider.sizeDelta = new Vector2(1f, 0f);
        divider.anchoredPosition = Vector2.zero;
        AddImage(divider, ctx.Gray200, null);

        BuildSidebarContent(sidebar, ctx);

        var main = CreateRect("MainArea", body);
        Stretch(main, 378f, 0f, 0f, 0f);
        BuildMainContent(main, ctx);
    }

    private static void BuildSidebarContent(RectTransform sidebar, UiContext ctx)
    {
        var content = CreateRect("Content", sidebar);
        Stretch(content, 12f, 14f, 14f, 16f);

        var layout = content.gameObject.AddComponent<VerticalLayoutGroup>();
        layout.padding = new RectOffset(0, 0, 8, 8);
        layout.spacing = 12f;
        layout.childAlignment = TextAnchor.UpperLeft;
        layout.childControlWidth = true;
        layout.childControlHeight = false;
        layout.childForceExpandHeight = false;
        layout.childForceExpandWidth = true;

        var labelObj = CreateRect("Label", content);
        AddLayoutSize(labelObj, -1f, 24f, 0f, 0f);
        var label = CreateText("Text", labelObj, "VEHICLE SELECTION", ctx.Font, 14, ctx.Gray500, FontStyles.Bold, TextAlignmentOptions.Left, false);
        Stretch(label.rectTransform, 0f, 0f, 0f, 0f);

        var search = CreateRect("SearchBox", content);
        AddLayoutSize(search, -1f, 54f, 0f, 0f);
        AddImage(search, ctx.White, ctx.RoundSprite);
        AddStroke(search, Hex("E2E7F1"));

        var searchIcon = CreateText("Icon", search, "⌕", ctx.Font, 18, ctx.Gray500, FontStyles.Bold, TextAlignmentOptions.Center, false);
        SetAnchor(searchIcon.rectTransform, new Vector2(0f, 0.5f), new Vector2(0f, 0.5f), new Vector2(0.5f, 0.5f));
        searchIcon.rectTransform.sizeDelta = new Vector2(24f, 24f);
        searchIcon.rectTransform.anchoredPosition = new Vector2(20f, 0f);

        var searchText = CreateText("Placeholder", search, "Search vehicle ID...", ctx.Font, 20, ctx.Gray500, FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(searchText.rectTransform, new Vector2(0f, 0f), new Vector2(1f, 1f), new Vector2(0f, 0.5f));
        searchText.rectTransform.offsetMin = new Vector2(44f, 0f);
        searchText.rectTransform.offsetMax = new Vector2(-16f, 0f);

        var list = CreateRect("VehicleList", content);
        AddLayoutSize(list, -1f, -1f, 1f, 0f);
        var listLayout = list.gameObject.AddComponent<VerticalLayoutGroup>();
        listLayout.spacing = 10f;
        listLayout.childAlignment = TextAnchor.UpperLeft;
        listLayout.childControlWidth = true;
        listLayout.childControlHeight = false;
        listLayout.childForceExpandHeight = false;
        listLayout.childForceExpandWidth = true;

        CreateVehicleItem(list, ctx, "RX-78 Prototype", "Session #4092 • Active", ctx.Green, true, 74f);
        CreateVehicleItem(list, ctx, "Tesla Model 3 Sim", "Last run: 2h ago", ctx.Gray400, false, 64f);
        CreateVehicleItem(list, ctx, "Waymo L4 Van", "Last run: 1d ago", ctx.Gray400, false, 64f);
        CreateVehicleItem(list, ctx, "NVIDIA BB8", "Error in Module 4", ctx.Red, false, 64f);

        var spacer = CreateRect("Spacer", content);
        AddLayoutSize(spacer, -1f, 0f, 1f, 0f);

        var newButton = CreateRect("NewSimulationButton", content);
        AddLayoutSize(newButton, -1f, 64f, 0f, 0f);
        AddImage(newButton, ctx.White, ctx.RoundSprite);
        AddStroke(newButton, Hex("DFE4EF"));

        var newText = CreateText("Text", newButton, "+  New Simulation", ctx.Font, 24, ctx.Gray700, FontStyles.Bold, TextAlignmentOptions.Center, false);
        Stretch(newText.rectTransform, 0f, 0f, 0f, 0f);
    }

    private static void CreateVehicleItem(RectTransform list, UiContext ctx, string title, string subtitle, Color dotColor, bool selected, float height)
    {
        var row = CreateRect(title.Replace(" ", string.Empty) + "Item", list);
        AddLayoutSize(row, -1f, height, 0f, 0f);

        if (selected)
        {
            AddImage(row, Hex("F6F8FF"), ctx.RoundSprite);
            AddStroke(row, Hex("C9D5FF"));
        }

        var icon = CreateRect("Icon", row);
        SetAnchor(icon, new Vector2(0f, 0.5f), new Vector2(0f, 0.5f), new Vector2(0.5f, 0.5f));
        icon.sizeDelta = new Vector2(24f, 24f);
        icon.anchoredPosition = new Vector2(18f, 0f);
        AddImage(icon, selected ? ctx.Blue800 : ctx.Gray200, ctx.RoundSprite);

        var titleText = CreateText("Title", row, title, ctx.Font, 31, selected ? ctx.Blue900 : Hex("4D5568"), FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(titleText.rectTransform, new Vector2(0f, 0.5f), new Vector2(1f, 0.5f), new Vector2(0f, 0.5f));
        titleText.rectTransform.offsetMin = new Vector2(48f, 2f);
        titleText.rectTransform.offsetMax = new Vector2(-28f, 22f);
        ConfigureSingleLineResponsive(titleText, 14f, 31f);

        var subText = CreateText("Subtitle", row, subtitle, ctx.Font, 18, selected ? Hex("7A85A0") : Hex("B0B8C8"), FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(subText.rectTransform, new Vector2(0f, 0.5f), new Vector2(1f, 0.5f), new Vector2(0f, 0.5f));
        subText.rectTransform.offsetMin = new Vector2(48f, -24f);
        subText.rectTransform.offsetMax = new Vector2(-28f, -2f);

        var dot = CreateRect("Dot", row);
        SetAnchor(dot, new Vector2(1f, 0.5f), new Vector2(1f, 0.5f), new Vector2(0.5f, 0.5f));
        dot.sizeDelta = new Vector2(8f, 8f);
        dot.anchoredPosition = new Vector2(-10f, 0f);
        AddImage(dot, dotColor, ctx.RoundSprite);
    }

    private static void BuildMainContent(RectTransform main, UiContext ctx)
    {
        var content = CreateRect("Content", main);
        Stretch(content, 28f, 20f, 26f, 24f);

        var layout = content.gameObject.AddComponent<VerticalLayoutGroup>();
        layout.spacing = 16f;
        layout.childAlignment = TextAnchor.UpperLeft;
        layout.childControlWidth = true;
        layout.childControlHeight = true;
        layout.childForceExpandHeight = false;
        layout.childForceExpandWidth = true;

        var header = CreateRect("HeaderRow", content);
        AddLayoutSize(header, -1f, 92f, 0f, 0f);
        BuildHeader(header, ctx);

        var kpi = CreateRect("KpiRow", content);
        AddLayoutSize(kpi, -1f, 152f, 0f, 0f);
        BuildKpiRow(kpi, ctx);

        var mainRow = CreateRect("MainRow", content);
        AddLayoutSize(mainRow, -1f, -1f, 1f, 0f);
        BuildMainRow(mainRow, ctx);
    }

    private static void BuildHeader(RectTransform header, UiContext ctx)
    {
        var layout = header.gameObject.AddComponent<HorizontalLayoutGroup>();
        layout.spacing = 16f;
        layout.childAlignment = TextAnchor.MiddleLeft;
        layout.childControlWidth = true;
        layout.childControlHeight = true;
        layout.childForceExpandWidth = false;
        layout.childForceExpandHeight = false;

        var titles = CreateRect("Titles", header);
        AddLayoutSize(titles, -1f, -1f, 1f, 0f);
        var titlesLayout = titles.gameObject.AddComponent<VerticalLayoutGroup>();
        titlesLayout.spacing = 8f;
        titlesLayout.childAlignment = TextAnchor.MiddleLeft;
        titlesLayout.childControlHeight = false;
        titlesLayout.childControlWidth = true;
        titlesLayout.childForceExpandHeight = false;
        titlesLayout.childForceExpandWidth = true;

        var title = CreateRect("Title", titles);
        AddLayoutSize(title, -1f, 46f, 0f, 0f);
        var titleText = CreateText("Text", title, "Training Analytics", ctx.Font, 34, ctx.Blue900, FontStyles.Bold, TextAlignmentOptions.Left, false);
        Stretch(titleText.rectTransform, 0f, 0f, 0f, 0f);

        var subtitle = CreateRect("Subtitle", titles);
        AddLayoutSize(subtitle, -1f, 24f, 0f, 0f);
        var subtitleText = CreateText("Text", subtitle, "Next-generation simulation environment performance overview.", ctx.Font, 13, ctx.Gray700, FontStyles.Normal, TextAlignmentOptions.Left, false);
        Stretch(subtitleText.rectTransform, 0f, 0f, 0f, 0f);

        var download = CreateRect("DownloadButton", header);
        AddLayoutSize(download, 280f, 56f, 0f, 0f);
        AddImage(download, ctx.Blue800, ctx.RoundSprite);

        var dIcon = CreateText("Icon", download, "↓", ctx.Font, 20, ctx.White, FontStyles.Bold, TextAlignmentOptions.Center, false);
        SetAnchor(dIcon.rectTransform, new Vector2(0f, 0.5f), new Vector2(0f, 0.5f), new Vector2(0.5f, 0.5f));
        dIcon.rectTransform.sizeDelta = new Vector2(24f, 24f);
        dIcon.rectTransform.anchoredPosition = new Vector2(24f, 0f);

        var dText = CreateText("Text", download, "Download Report", ctx.Font, 14, ctx.White, FontStyles.Bold, TextAlignmentOptions.Center, false);
        Stretch(dText.rectTransform, 32f, 0f, 8f, 0f);
    }

    private static void BuildKpiRow(RectTransform row, UiContext ctx)
    {
        var layout = row.gameObject.AddComponent<HorizontalLayoutGroup>();
        layout.spacing = 16f;
        layout.childAlignment = TextAnchor.UpperLeft;
        layout.childControlHeight = true;
        layout.childControlWidth = true;
        layout.childForceExpandHeight = true;
        layout.childForceExpandWidth = true;

        BuildKpiCardSuccess(CreateRect("CardSuccess", row), ctx);
        BuildKpiCardDistance(CreateRect("CardDistance", row), ctx);
        BuildKpiCardSpeed(CreateRect("CardSpeed", row), ctx);
    }

    private static void BuildKpiCardSuccess(RectTransform card, UiContext ctx)
    {
        AddLayoutSize(card, -1f, -1f, 1f, 1f);
        AddImage(card, ctx.White, ctx.RoundSprite);
        AddStroke(card, ctx.Gray200);

        var title = CreateText("Title", card, "SUCCESS RATE", ctx.Font, 14, ctx.Gray700, FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(title.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0f, 1f));
        title.rectTransform.offsetMin = new Vector2(18f, -30f);
        title.rectTransform.offsetMax = new Vector2(-56f, -8f);

        var value = CreateText("Value", card, "94.2%", ctx.Font, 42, Hex("1A243B"), FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(value.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0f, 1f));
        value.rectTransform.offsetMin = new Vector2(18f, -94f);
        value.rectTransform.offsetMax = new Vector2(-120f, -30f);

        var delta = CreateText("Delta", card, "↗ +2.4%", ctx.Font, 16, ctx.Green, FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(delta.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0f, 1f));
        delta.rectTransform.offsetMin = new Vector2(146f, -90f);
        delta.rectTransform.offsetMax = new Vector2(-24f, -42f);

        var barBg = CreateRect("ProgressBg", card);
        SetAnchor(barBg, new Vector2(0f, 0f), new Vector2(1f, 0f), new Vector2(0f, 0f));
        barBg.sizeDelta = new Vector2(-36f, 8f);
        barBg.anchoredPosition = new Vector2(18f, 28f);
        AddImage(barBg, ctx.Gray150, ctx.RoundSprite);

        var barFill = CreateRect("ProgressFill", barBg);
        SetAnchor(barFill, new Vector2(0f, 0f), new Vector2(0f, 1f), new Vector2(0f, 0.5f));
        barFill.sizeDelta = new Vector2(360f, 0f);
        AddImage(barFill, ctx.Blue800, ctx.RoundSprite);

        var badge = CreateRect("Badge", card);
        SetAnchor(badge, new Vector2(1f, 1f), new Vector2(1f, 1f), new Vector2(1f, 1f));
        badge.sizeDelta = new Vector2(50f, 50f);
        badge.anchoredPosition = new Vector2(-18f, -18f);
        AddImage(badge, Hex("F0F2F8"), ctx.RoundSprite);
        var check = CreateText("Icon", badge, "✓", ctx.Font, 24, ctx.Gray400, FontStyles.Bold, TextAlignmentOptions.Center, false);
        Stretch(check.rectTransform, 0f, 0f, 0f, 0f);
    }

    private static void BuildKpiCardDistance(RectTransform card, UiContext ctx)
    {
        AddLayoutSize(card, -1f, -1f, 1f, 1f);
        AddImage(card, ctx.White, ctx.RoundSprite);
        AddStroke(card, ctx.Gray200);

        var title = CreateText("Title", card, "DISTANCE TRAVELED", ctx.Font, 14, ctx.Gray700, FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(title.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0f, 1f));
        title.rectTransform.offsetMin = new Vector2(18f, -30f);
        title.rectTransform.offsetMax = new Vector2(-56f, -8f);

        var value = CreateText("Value", card, "1,248", ctx.Font, 42, Hex("1A243B"), FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(value.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0f, 1f));
        value.rectTransform.offsetMin = new Vector2(18f, -96f);
        value.rectTransform.offsetMax = new Vector2(-128f, -30f);

        var unit = CreateText("Unit", card, "km", ctx.Font, 24, ctx.Gray700, FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(unit.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0f, 1f));
        unit.rectTransform.offsetMin = new Vector2(152f, -92f);
        unit.rectTransform.offsetMax = new Vector2(-18f, -34f);

        var desc = CreateText("Desc", card, "Total simulation distance this session", ctx.Font, 12, ctx.Gray500, FontStyles.Normal, TextAlignmentOptions.Left, false);
        SetAnchor(desc.rectTransform, new Vector2(0f, 0f), new Vector2(1f, 0f), new Vector2(0f, 0f));
        desc.rectTransform.offsetMin = new Vector2(18f, 14f);
        desc.rectTransform.offsetMax = new Vector2(-18f, 34f);

        var badge = CreateRect("Badge", card);
        SetAnchor(badge, new Vector2(1f, 1f), new Vector2(1f, 1f), new Vector2(1f, 1f));
        badge.sizeDelta = new Vector2(50f, 50f);
        badge.anchoredPosition = new Vector2(-18f, -18f);
        AddImage(badge, Hex("F0F2F8"), ctx.RoundSprite);
        var check = CreateText("Icon", badge, "∿", ctx.Font, 24, ctx.Gray400, FontStyles.Bold, TextAlignmentOptions.Center, false);
        Stretch(check.rectTransform, 0f, 0f, 0f, 0f);
    }

    private static void BuildKpiCardSpeed(RectTransform card, UiContext ctx)
    {
        AddLayoutSize(card, -1f, -1f, 1f, 1f);
        AddImage(card, ctx.White, ctx.RoundSprite);
        AddStroke(card, ctx.Gray200);

        var title = CreateText("Title", card, "AVERAGE SPEED", ctx.Font, 14, ctx.Gray700, FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(title.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0f, 1f));
        title.rectTransform.offsetMin = new Vector2(18f, -30f);
        title.rectTransform.offsetMax = new Vector2(-56f, -8f);

        var value = CreateText("Value", card, "42.5", ctx.Font, 42, Hex("1A243B"), FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(value.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0f, 1f));
        value.rectTransform.offsetMin = new Vector2(18f, -96f);
        value.rectTransform.offsetMax = new Vector2(-128f, -30f);

        var unit = CreateText("Unit", card, "km/h", ctx.Font, 24, ctx.Gray700, FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(unit.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0f, 1f));
        unit.rectTransform.offsetMin = new Vector2(170f, -92f);
        unit.rectTransform.offsetMax = new Vector2(-18f, -34f);

        var desc = CreateText("Desc", card, "Maintained across urban scenarios", ctx.Font, 12, ctx.Gray500, FontStyles.Normal, TextAlignmentOptions.Left, false);
        SetAnchor(desc.rectTransform, new Vector2(0f, 0f), new Vector2(1f, 0f), new Vector2(0f, 0f));
        desc.rectTransform.offsetMin = new Vector2(18f, 14f);
        desc.rectTransform.offsetMax = new Vector2(-18f, 34f);

        var badge = CreateRect("Badge", card);
        SetAnchor(badge, new Vector2(1f, 1f), new Vector2(1f, 1f), new Vector2(1f, 1f));
        badge.sizeDelta = new Vector2(50f, 50f);
        badge.anchoredPosition = new Vector2(-18f, -18f);
        AddImage(badge, Hex("F0F2F8"), ctx.RoundSprite);
        var check = CreateText("Icon", badge, "◔", ctx.Font, 22, ctx.Gray400, FontStyles.Bold, TextAlignmentOptions.Center, false);
        Stretch(check.rectTransform, 0f, 0f, 0f, 0f);
    }

    private static void BuildMainRow(RectTransform row, UiContext ctx)
    {
        var layout = row.gameObject.AddComponent<HorizontalLayoutGroup>();
        layout.spacing = 16f;
        layout.childAlignment = TextAnchor.UpperLeft;
        layout.childControlHeight = true;
        layout.childControlWidth = true;
        layout.childForceExpandHeight = true;
        layout.childForceExpandWidth = true;

        var chart = CreateRect("ChartCard", row);
        AddLayoutSize(chart, -1f, -1f, 1.9f, 1f);
        BuildChartCard(chart, ctx);

        var events = CreateRect("EventsCard", row);
        AddLayoutSize(events, -1f, -1f, 1.0f, 1f);
        BuildEventsCard(events, ctx);
    }

    private static void BuildChartCard(RectTransform card, UiContext ctx)
    {
        AddImage(card, ctx.White, ctx.RoundSprite);
        AddStroke(card, ctx.Gray200);

        var title = CreateText("Title", card, "Learning Progress & Loss", ctx.Font, 18, ctx.Blue900, FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(title.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0f, 1f));
        title.rectTransform.offsetMin = new Vector2(24f, -46f);
        title.rectTransform.offsetMax = new Vector2(-220f, -12f);

        var dropdown = CreateRect("Range", card);
        SetAnchor(dropdown, new Vector2(1f, 1f), new Vector2(1f, 1f), new Vector2(1f, 1f));
        dropdown.sizeDelta = new Vector2(140f, 36f);
        dropdown.anchoredPosition = new Vector2(-20f, -28f);
        AddImage(dropdown, Hex("F7F9FC"), ctx.RoundSprite);
        AddStroke(dropdown, ctx.Gray300);

        var dText = CreateText("Text", dropdown, "Last 24 Hours", ctx.Font, 12, ctx.Gray700, FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(dText.rectTransform, new Vector2(0f, 0f), new Vector2(1f, 1f), new Vector2(0f, 0.5f));
        dText.rectTransform.offsetMin = new Vector2(10f, 0f);
        dText.rectTransform.offsetMax = new Vector2(-20f, 0f);

        var dArrow = CreateText("Arrow", dropdown, "⌄", ctx.Font, 14, ctx.Gray500, FontStyles.Bold, TextAlignmentOptions.Center, false);
        SetAnchor(dArrow.rectTransform, new Vector2(1f, 0.5f), new Vector2(1f, 0.5f), new Vector2(0.5f, 0.5f));
        dArrow.rectTransform.sizeDelta = new Vector2(16f, 16f);
        dArrow.rectTransform.anchoredPosition = new Vector2(-10f, 0f);

        var plot = CreateRect("Plot", card);
        Stretch(plot, 24f, 22f, 22f, 68f);
        AddImage(plot, Hex("FBFCFE"), null);

        for (int i = 0; i < 5; i++)
        {
            float y = Mathf.Lerp(0.08f, 0.92f, i / 4f);
            var grid = CreateRect("Grid" + i, plot);
            SetAnchor(grid, new Vector2(0f, y), new Vector2(1f, y), new Vector2(0.5f, 0.5f));
            grid.sizeDelta = new Vector2(0f, 1f);
            AddImage(grid, Hex("ECF0F7"), null);
        }

        var fill = CreateRect("AreaFill", plot);
        SetAnchor(fill, new Vector2(0f, 0.05f), new Vector2(1f, 0.55f), new Vector2(0.5f, 0.5f));
        fill.offsetMin = new Vector2(0f, 0f);
        fill.offsetMax = new Vector2(0f, 0f);
        AddImage(fill, new Color(ctx.Blue800.r, ctx.Blue800.g, ctx.Blue800.b, 0.06f), null);

        QueueLine(plot,
            new[] {
                new Vector2(0.03f, 0.88f), new Vector2(0.18f, 0.62f), new Vector2(0.34f, 0.44f),
                new Vector2(0.52f, 0.30f), new Vector2(0.69f, 0.21f), new Vector2(0.86f, 0.15f), new Vector2(0.97f, 0.11f)
            },
            ctx.Blue800,
            2.6f,
            false);

        QueueLine(plot,
            new[] {
                new Vector2(0.03f, 0.12f), new Vector2(0.18f, 0.17f), new Vector2(0.34f, 0.26f),
                new Vector2(0.52f, 0.40f), new Vector2(0.69f, 0.56f), new Vector2(0.82f, 0.73f), new Vector2(0.97f, 0.82f)
            },
            Hex("9FB0E3"),
            2.2f,
            true);

        var tooltip = CreateRect("Tooltip", plot);
        SetAnchor(tooltip, new Vector2(0.56f, 0.35f), new Vector2(0.56f, 0.35f), new Vector2(0.5f, 0.5f));
        tooltip.sizeDelta = new Vector2(70f, 24f);
        AddImage(tooltip, ctx.Blue800, ctx.RoundSprite);
        var tip = CreateText("Text", tooltip, "Loss: 0.22", ctx.Font, 11, ctx.White, FontStyles.Bold, TextAlignmentOptions.Center, false);
        Stretch(tip.rectTransform, 0f, 0f, 0f, 0f);

        var xLabels = CreateRect("XAxis", plot);
        SetAnchor(xLabels, new Vector2(0f, 0f), new Vector2(1f, 0f), new Vector2(0.5f, 0f));
        xLabels.sizeDelta = new Vector2(0f, 16f);
        xLabels.anchoredPosition = new Vector2(0f, -14f);

        string[] labels = { "00:00", "04:00", "08:00", "12:00", "16:00", "20:00" };
        for (int i = 0; i < labels.Length; i++)
        {
            float x = i / (float)(labels.Length - 1);
            var lb = CreateText("Label" + i, xLabels, labels[i], ctx.Font, 9, ctx.Gray500, FontStyles.Bold, TextAlignmentOptions.Center, false);
            SetAnchor(lb.rectTransform, new Vector2(x, 0.5f), new Vector2(x, 0.5f), new Vector2(0.5f, 0.5f));
            lb.rectTransform.sizeDelta = new Vector2(40f, 14f);
        }
    }

    private static void BuildEventsCard(RectTransform card, UiContext ctx)
    {
        AddImage(card, ctx.White, ctx.RoundSprite);
        AddStroke(card, ctx.Gray200);

        var title = CreateText("Title", card, "Critical Events", ctx.Font, 18, ctx.Blue900, FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(title.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0f, 1f));
        title.rectTransform.offsetMin = new Vector2(18f, -42f);
        title.rectTransform.offsetMax = new Vector2(-90f, -12f);

        var badge = CreateRect("NewBadge", card);
        SetAnchor(badge, new Vector2(1f, 1f), new Vector2(1f, 1f), new Vector2(1f, 1f));
        badge.sizeDelta = new Vector2(54f, 22f);
        badge.anchoredPosition = new Vector2(-18f, -24f);
        AddImage(badge, Hex("FFF1F2"), ctx.RoundSprite);
        AddStroke(badge, Hex("FFD7DB"));
        var badgeText = CreateText("Text", badge, "3 New", ctx.Font, 11, Hex("F87171"), FontStyles.Bold, TextAlignmentOptions.Center, false);
        Stretch(badgeText.rectTransform, 0f, 0f, 0f, 0f);

        var list = CreateRect("List", card);
        Stretch(list, 18f, 14f, 18f, 54f);
        var layout = list.gameObject.AddComponent<VerticalLayoutGroup>();
        layout.spacing = 10f;
        layout.childAlignment = TextAnchor.UpperLeft;
        layout.childControlWidth = true;
        layout.childControlHeight = false;
        layout.childForceExpandHeight = false;
        layout.childForceExpandWidth = true;

        CreateEventItem(list, ctx, "Collision Detected", "Front bumper impact with static obstacle.", "14:02:45", Hex("FEF2F2"), Hex("FCA5A5"), "▲");
        CreateEventItem(list, ctx, "Near Miss", "Proximity warning < 0.5m with pedestrian actor.", "13:45:12", Hex("FFFBEB"), Hex("FCD34D"), "✕");
        CreateEventItem(list, ctx, "Lane Deviation", "Crossed solid line marker without signaling.", "13:10:05", Hex("F8FAFF"), Hex("C7D2FE"), "⇄");
        CreateEventItem(list, ctx, "Sudden Braking", "Deceleration > 4.5g detected.", "12:55:30", Hex("F8FAFF"), Hex("C7D2FE"), "◔");
    }

    private static void CreateEventItem(RectTransform parent, UiContext ctx, string title, string desc, string time, Color bg, Color border, string icon)
    {
        var row = CreateRect(title.Replace(" ", string.Empty) + "Item", parent);
        AddLayoutSize(row, -1f, 94f, 0f, 0f);
        AddImage(row, bg, ctx.RoundSprite);
        AddStroke(row, border);

        var iconText = CreateText("Icon", row, icon, ctx.Font, 14, border, FontStyles.Bold, TextAlignmentOptions.Center, false);
        SetAnchor(iconText.rectTransform, new Vector2(0f, 1f), new Vector2(0f, 1f), new Vector2(0.5f, 1f));
        iconText.rectTransform.sizeDelta = new Vector2(24f, 24f);
        iconText.rectTransform.anchoredPosition = new Vector2(18f, -18f);

        var titleText = CreateText("Title", row, title, ctx.Font, 14, Hex("2F3545"), FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(titleText.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0f, 1f));
        titleText.rectTransform.offsetMin = new Vector2(40f, -28f);
        titleText.rectTransform.offsetMax = new Vector2(-78f, -8f);

        var timeText = CreateText("Time", row, time, ctx.Font, 11, ctx.Gray500, FontStyles.Bold, TextAlignmentOptions.Right, false);
        SetAnchor(timeText.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(1f, 1f));
        timeText.rectTransform.offsetMin = new Vector2(40f, -28f);
        timeText.rectTransform.offsetMax = new Vector2(-12f, -8f);

        var descText = CreateText("Description", row, desc, ctx.Font, 11, ctx.Gray700, FontStyles.Normal, TextAlignmentOptions.Left, true);
        SetAnchor(descText.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0f, 1f));
        descText.rectTransform.offsetMin = new Vector2(40f, -52f);
        descText.rectTransform.offsetMax = new Vector2(-12f, -30f);

        var replay = CreateText("Replay", row, "◉ Replay Event", ctx.Font, 11, ctx.Blue800, FontStyles.Bold, TextAlignmentOptions.Left, false);
        SetAnchor(replay.rectTransform, new Vector2(0f, 0f), new Vector2(1f, 0f), new Vector2(0f, 0f));
        replay.rectTransform.offsetMin = new Vector2(40f, 6f);
        replay.rectTransform.offsetMax = new Vector2(-12f, 22f);
    }

    private static void BuildBottomBar(RectTransform root, UiContext ctx)
    {
        var bottom = CreateRect("BottomBar", root);
        AnchorBottom(bottom, 42f);
        AddImage(bottom, ctx.White, null);

        var line = CreateRect("TopLine", bottom);
        AnchorTop(line, 1f);
        AddImage(line, ctx.Gray200, null);

        var content = CreateRect("Content", bottom);
        Stretch(content, 20f, 8f, 20f, 8f);

        var left = CreateRect("Left", content);
        SetAnchor(left, new Vector2(0f, 0.5f), new Vector2(0f, 0.5f), new Vector2(0f, 0.5f));
        left.sizeDelta = new Vector2(420f, 24f);
        var leftLayout = left.gameObject.AddComponent<HorizontalLayoutGroup>();
        leftLayout.spacing = 14f;
        leftLayout.childAlignment = TextAnchor.MiddleLeft;
        leftLayout.childControlWidth = false;
        leftLayout.childControlHeight = false;
        leftLayout.childForceExpandHeight = false;
        leftLayout.childForceExpandWidth = false;

        var dot = CreateText("Dot", left, "•", ctx.Font, 14, ctx.Green, FontStyles.Bold, TextAlignmentOptions.Center, false);
        AddLayoutSize(dot.rectTransform, 14f, 18f, 0f, 0f);
        CreateBottomText(left, ctx, "System Online");
        CreateBottomText(left, ctx, "|", ctx.Gray400, 12);
        CreateBottomText(left, ctx, "v2.4.1 (Stable)");

        var right = CreateRect("Right", content);
        SetAnchor(right, new Vector2(1f, 0.5f), new Vector2(1f, 0.5f), new Vector2(1f, 0.5f));
        right.sizeDelta = new Vector2(520f, 24f);
        var rightLayout = right.gameObject.AddComponent<HorizontalLayoutGroup>();
        rightLayout.spacing = 20f;
        rightLayout.childAlignment = TextAnchor.MiddleRight;
        rightLayout.childControlWidth = false;
        rightLayout.childControlHeight = false;
        rightLayout.childForceExpandHeight = false;
        rightLayout.childForceExpandWidth = false;

        CreateBottomText(right, ctx, "Engine: Unity 2023");
        CreateBottomText(right, ctx, "Server: Localhost:8080");
        CreateBottomText(right, ctx, "↗ 12ms");
    }

    private static void CreateBottomText(RectTransform parent, UiContext ctx, string text, Color? color = null, int size = 12)
    {
        var t = CreateText("Text", parent, text, ctx.Font, size, color ?? ctx.Gray700, FontStyles.Bold, TextAlignmentOptions.Center, false);
        var le = t.gameObject.AddComponent<LayoutElement>();
        le.preferredWidth = Mathf.Max(16f, t.preferredWidth + 2f);
        le.preferredHeight = 18f;
    }

    private static RectTransform CreateRect(string name, Transform parent)
    {
        var go = new GameObject(name, typeof(RectTransform));
        var rt = go.GetComponent<RectTransform>();
        rt.SetParent(parent, false);
        rt.localScale = Vector3.one;
        rt.localRotation = Quaternion.identity;
        return rt;
    }

    private static Image AddImage(RectTransform rt, Color color, Sprite sprite)
    {
        var img = rt.gameObject.AddComponent<Image>();
        img.color = color;
        if (sprite != null)
        {
            img.sprite = sprite;
            img.type = Image.Type.Sliced;
        }
        return img;
    }

    private static void AddStroke(RectTransform rt, Color color)
    {
        var outline = rt.gameObject.AddComponent<Outline>();
        outline.effectColor = color;
        outline.effectDistance = new Vector2(1f, -1f);
        outline.useGraphicAlpha = true;
    }

    private static TextMeshProUGUI CreateText(string name, RectTransform parent, string text, TMP_FontAsset font, int fontSize, Color color, FontStyles style, TextAlignmentOptions align, bool wrap)
    {
        var rt = CreateRect(name, parent);
        var tmp = rt.gameObject.AddComponent<TextMeshProUGUI>();
        tmp.text = text;
        tmp.font = font;
        tmp.fontSize = fontSize;
        tmp.color = color;
        tmp.fontStyle = style;
        tmp.alignment = align;
        tmp.enableWordWrapping = wrap;
        tmp.overflowMode = wrap ? TextOverflowModes.Truncate : TextOverflowModes.Overflow;
        tmp.richText = true;
        tmp.raycastTarget = false;
        return tmp;
    }

    private static void ConfigureSingleLineResponsive(TextMeshProUGUI tmp, float minSize, float maxSize)
    {
        if (tmp == null)
        {
            return;
        }

        tmp.enableWordWrapping = false;
        tmp.overflowMode = TextOverflowModes.Ellipsis;
        tmp.enableAutoSizing = true;
        tmp.fontSizeMin = minSize;
        tmp.fontSizeMax = maxSize;
    }

    private static void Stretch(RectTransform rt, float left, float bottom, float right, float top)
    {
        rt.anchorMin = Vector2.zero;
        rt.anchorMax = Vector2.one;
        rt.pivot = new Vector2(0.5f, 0.5f);
        rt.offsetMin = new Vector2(left, bottom);
        rt.offsetMax = new Vector2(-right, -top);
    }

    private static void AnchorTop(RectTransform rt, float height)
    {
        SetAnchor(rt, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0.5f, 1f));
        rt.sizeDelta = new Vector2(0f, height);
        rt.anchoredPosition = Vector2.zero;
    }

    private static void AnchorBottom(RectTransform rt, float height)
    {
        SetAnchor(rt, new Vector2(0f, 0f), new Vector2(1f, 0f), new Vector2(0.5f, 0f));
        rt.sizeDelta = new Vector2(0f, height);
        rt.anchoredPosition = Vector2.zero;
    }

    private static void SetAnchor(RectTransform rt, Vector2 min, Vector2 max, Vector2 pivot)
    {
        rt.anchorMin = min;
        rt.anchorMax = max;
        rt.pivot = pivot;
    }

    private static void AddLayoutSize(RectTransform rt, float prefW, float prefH, float flexW, float flexH)
    {
        var le = rt.GetComponent<LayoutElement>();
        if (le == null)
        {
            le = rt.gameObject.AddComponent<LayoutElement>();
        }

        if (prefW >= 0f)
        {
            le.preferredWidth = prefW;
        }

        if (prefH >= 0f)
        {
            le.preferredHeight = prefH;
        }

        le.flexibleWidth = flexW;
        le.flexibleHeight = flexH;
    }

    private static void QueueLine(RectTransform parent, Vector2[] points, Color color, float thickness, bool dashed)
    {
        QueuedLines.Add(new LineInstruction
        {
            Parent = parent,
            Points = points,
            Color = color,
            Thickness = thickness,
            Dashed = dashed
        });
    }

    private static void DrawQueuedLines()
    {
        foreach (var line in QueuedLines)
        {
            if (line.Parent == null || line.Points == null || line.Points.Length < 2)
            {
                continue;
            }

            var rect = line.Parent.rect;
            if (rect.width < 2f || rect.height < 2f)
            {
                continue;
            }

            for (int i = 0; i < line.Points.Length - 1; i++)
            {
                var p1 = ToLocal(rect, line.Points[i]);
                var p2 = ToLocal(rect, line.Points[i + 1]);
                if (line.Dashed)
                {
                    DrawDashedSegment(line.Parent, p1, p2, line.Color, line.Thickness, 10f, 8f);
                }
                else
                {
                    DrawSegment(line.Parent, p1, p2, line.Color, line.Thickness);
                }
            }
        }

        QueuedLines.Clear();
    }

    private static Vector2 ToLocal(Rect rect, Vector2 normalized)
    {
        return new Vector2(
            rect.xMin + (rect.width * Mathf.Clamp01(normalized.x)),
            rect.yMin + (rect.height * Mathf.Clamp01(normalized.y))
        );
    }

    private static void DrawDashedSegment(RectTransform parent, Vector2 a, Vector2 b, Color color, float thickness, float dash, float gap)
    {
        var dir = (b - a);
        var len = dir.magnitude;
        if (len <= 0.01f)
        {
            return;
        }

        dir /= len;
        float cursor = 0f;
        while (cursor < len)
        {
            float seg = Mathf.Min(dash, len - cursor);
            var s = a + dir * cursor;
            var e = s + dir * seg;
            DrawSegment(parent, s, e, color, thickness);
            cursor += dash + gap;
        }
    }

    private static void DrawSegment(RectTransform parent, Vector2 a, Vector2 b, Color color, float thickness)
    {
        var seg = CreateRect("Line", parent);
        var img = AddImage(seg, color, null);
        img.raycastTarget = false;

        var d = b - a;
        float len = d.magnitude;
        if (len < 0.01f)
        {
            len = 0.01f;
        }

        seg.sizeDelta = new Vector2(len, thickness);
        seg.anchoredPosition = (a + b) * 0.5f;
        float angle = Mathf.Atan2(d.y, d.x) * Mathf.Rad2Deg;
        seg.localRotation = Quaternion.Euler(0f, 0f, angle);
    }

    private static TMP_FontAsset LoadFont()
    {
#if UNITY_EDITOR
        var font = AssetDatabase.LoadAssetAtPath<TMP_FontAsset>(FallbackFontPath);
        if (font != null)
        {
            return font;
        }
#endif
        return TMP_Settings.defaultFontAsset;
    }

    private static Sprite LoadRoundSprite()
    {
        return null;
    }

    private static Color Hex(string hex)
    {
        if (!hex.StartsWith("#"))
        {
            hex = "#" + hex;
        }

        ColorUtility.TryParseHtmlString(hex, out var c);
        return c;
    }

    private struct UiContext
    {
        public TMP_FontAsset Font;
        public Sprite RoundSprite;

        public Color Blue900;
        public Color Blue800;
        public Color Blue700;
        public Color Blue300;

        public Color Gray900;
        public Color Gray700;
        public Color Gray600;
        public Color Gray500;
        public Color Gray400;
        public Color Gray300;
        public Color Gray200;
        public Color Gray150;
        public Color Gray100;

        public Color White;
        public Color Green;
        public Color Yellow;
        public Color Red;
    }
}
