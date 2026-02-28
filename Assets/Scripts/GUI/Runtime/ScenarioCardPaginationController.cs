using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace GUI.Runtime
{
    [DisallowMultipleComponent]
    public class ScenarioCardPaginationController : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private RectTransform cardsArea;
        [SerializeField] private RectTransform cardRow;
        [SerializeField] private Button leftArrowButton;
        [SerializeField] private Button rightArrowButton;
        [SerializeField] private RectTransform paginationBar;

        [Header("Paging")]
        [SerializeField] private int visibleCardCount = 3;
        [SerializeField] private float transitionDuration = 0.22f;
        [SerializeField] private float transitionOffset = 96f;
        [SerializeField] private bool useUnscaledTime = true;

        [Header("Pagination Style")]
        [SerializeField] private float paginationBottomInset = 72f;
        [SerializeField] private float pageItemWidth = 52f;
        [SerializeField] private float pageItemHeight = 44f;
        [SerializeField] private Color pageTextColor = new Color32(0x5F, 0x66, 0x78, 0xFF);
        [SerializeField] private Color pageActiveTextColor = Color.white;
        [SerializeField] private Color pageActiveBackgroundColor = new Color32(0x0A, 0x4A, 0x8A, 0xFF);

        private readonly List<RectTransform> _cards = new List<RectTransform>();
        private readonly List<CanvasGroup> _cardGroups = new List<CanvasGroup>();
        private readonly List<Button> _pageButtons = new List<Button>();

        private Coroutine _transitionRoutine;
        private Font _uiFont;
        private Sprite _pillSprite;
        private int _currentPage;
        private int _pageCount = 1;
        private bool _isAnimating;
        private bool _initialized;

        private void Awake()
        {
            AutoWire();
            EnsureArrowButtons();
            EnsurePaginationBar();
            ResolveStyleAssets();
            CacheCards();
            ApplyPageImmediate(0);
            _initialized = true;
        }

        private void OnEnable()
        {
            if (!_initialized)
            {
                AutoWire();
                EnsureArrowButtons();
                ResolveStyleAssets();
                _initialized = true;
            }

            BindArrowEvents();
            EnsurePaginationBar();
            CacheCards();
            ApplyPageImmediate(_currentPage);
            RebuildPagination();
            UpdateArrowState();
        }

        private void OnDisable()
        {
            UnbindArrowEvents();

            if (_transitionRoutine != null)
            {
                StopCoroutine(_transitionRoutine);
                _transitionRoutine = null;
            }

            _isAnimating = false;
        }

        private void AutoWire()
        {
            if (cardsArea == null)
                cardsArea = transform.Find("MainSection/CardsArea") as RectTransform;

            if (cardRow == null && cardsArea != null)
                cardRow = cardsArea.Find("CardRow") as RectTransform;

            if (leftArrowButton == null && cardsArea != null)
            {
                var left = cardsArea.Find("LeftArrow");
                if (left != null)
                    leftArrowButton = left.GetComponent<Button>();
            }

            if (rightArrowButton == null && cardsArea != null)
            {
                var right = cardsArea.Find("RightArrow");
                if (right != null)
                    rightArrowButton = right.GetComponent<Button>();
            }

            if (paginationBar == null)
            {
                var existing = transform.Find("MainSection/PaginationBar") as RectTransform;
                if (existing != null)
                    paginationBar = existing;
            }
        }

        private void EnsureArrowButtons()
        {
            if (cardsArea == null)
                return;

            if (leftArrowButton == null)
            {
                var left = cardsArea.Find("LeftArrow");
                if (left != null)
                    leftArrowButton = EnsureArrowButton(left.gameObject);
            }
            else
            {
                ConfigureArrowButton(leftArrowButton);
            }

            if (rightArrowButton == null)
            {
                var right = cardsArea.Find("RightArrow");
                if (right != null)
                    rightArrowButton = EnsureArrowButton(right.gameObject);
            }
            else
            {
                ConfigureArrowButton(rightArrowButton);
            }
        }

        private static Button EnsureArrowButton(GameObject target)
        {
            if (target == null)
                return null;

            var button = target.GetComponent<Button>();
            if (button == null)
                button = target.AddComponent<Button>();

            ConfigureArrowButton(button);
            return button;
        }

        private static void ConfigureArrowButton(Button button)
        {
            if (button == null)
                return;

            var image = button.GetComponent<Image>();
            if (button.targetGraphic == null && image != null)
                button.targetGraphic = image;

            button.transition = Selectable.Transition.ColorTint;
            var colors = button.colors;
            colors.normalColor = Color.white;
            colors.highlightedColor = new Color(1f, 1f, 1f, 0.92f);
            colors.pressedColor = new Color(1f, 1f, 1f, 0.8f);
            colors.disabledColor = new Color(1f, 1f, 1f, 0.35f);
            colors.colorMultiplier = 1f;
            colors.fadeDuration = 0.08f;
            button.colors = colors;
        }

        private void EnsurePaginationBar()
        {
            if (paginationBar == null)
            {
                var mainSection = transform.Find("MainSection") as RectTransform;
                if (mainSection == null)
                    return;

                var go = new GameObject("PaginationBar", typeof(RectTransform), typeof(HorizontalLayoutGroup));
                paginationBar = go.GetComponent<RectTransform>();
                paginationBar.SetParent(mainSection, false);
            }

            paginationBar.anchorMin = new Vector2(0.5f, 0f);
            paginationBar.anchorMax = new Vector2(0.5f, 0f);
            paginationBar.pivot = new Vector2(0.5f, 0.5f);
            paginationBar.sizeDelta = new Vector2(560f, 54f);

            var targetY = paginationBottomInset;
            var parentRect = paginationBar.parent as RectTransform;
            if (cardsArea != null && parentRect != null)
            {
                var cardsAreaBottom = cardsArea.anchoredPosition.y - (cardsArea.rect.height * 0.5f);
                var parentBottom = -parentRect.rect.height * 0.5f;
                var cardsBottomFromParentBottom = cardsAreaBottom - parentBottom;
                var belowCardsY = cardsBottomFromParentBottom - (pageItemHeight * 0.5f) - 18f;
                targetY = Mathf.Max(paginationBottomInset, belowCardsY);
            }

            paginationBar.anchoredPosition = new Vector2(0f, targetY);

            var layout = paginationBar.GetComponent<HorizontalLayoutGroup>();
            if (layout == null)
                layout = paginationBar.gameObject.AddComponent<HorizontalLayoutGroup>();

            layout.spacing = 12f;
            layout.childAlignment = TextAnchor.MiddleCenter;
            layout.childControlWidth = false;
            layout.childControlHeight = false;
            layout.childForceExpandWidth = false;
            layout.childForceExpandHeight = false;
            layout.padding = new RectOffset(0, 0, 0, 0);
        }

        private void ResolveStyleAssets()
        {
            if (_uiFont == null)
            {
                var text = GetComponentInChildren<Text>(true);
                if (text != null && text.font != null)
                    _uiFont = text.font;
            }

            if (_uiFont == null)
            {
                _uiFont = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
                if (_uiFont == null)
                    _uiFont = Font.CreateDynamicFontFromOSFont("Arial", 16);
            }

            if (_pillSprite == null)
            {
                var sourceImage = cardsArea != null ? cardsArea.GetComponentInChildren<Image>(true) : null;
                if (sourceImage != null && sourceImage.sprite != null)
                    _pillSprite = sourceImage.sprite;
            }
        }

        private void BindArrowEvents()
        {
            if (leftArrowButton != null)
            {
                leftArrowButton.onClick.RemoveListener(GoPreviousPage);
                leftArrowButton.onClick.AddListener(GoPreviousPage);
            }

            if (rightArrowButton != null)
            {
                rightArrowButton.onClick.RemoveListener(GoNextPage);
                rightArrowButton.onClick.AddListener(GoNextPage);
            }
        }

        private void UnbindArrowEvents()
        {
            if (leftArrowButton != null)
                leftArrowButton.onClick.RemoveListener(GoPreviousPage);

            if (rightArrowButton != null)
                rightArrowButton.onClick.RemoveListener(GoNextPage);
        }

        private void CacheCards()
        {
            _cards.Clear();
            _cardGroups.Clear();

            if (cardRow == null)
                return;

            for (var i = 0; i < cardRow.childCount; i++)
            {
                var child = cardRow.GetChild(i) as RectTransform;
                if (child == null)
                    continue;

                if (!child.name.StartsWith("Card_", System.StringComparison.Ordinal))
                    continue;

                _cards.Add(child);
                _cardGroups.Add(EnsureCanvasGroup(child));
            }

            var pageSize = Mathf.Max(1, visibleCardCount);
            _pageCount = Mathf.Max(1, Mathf.CeilToInt((float)_cards.Count / pageSize));
            _currentPage = Mathf.Clamp(_currentPage, 0, _pageCount - 1);
        }

        private static CanvasGroup EnsureCanvasGroup(RectTransform target)
        {
            var group = target.GetComponent<CanvasGroup>();
            if (group == null)
                group = target.gameObject.AddComponent<CanvasGroup>();

            return group;
        }

        private void GoPreviousPage()
        {
            GoToPage(_currentPage - 1);
        }

        private void GoNextPage()
        {
            GoToPage(_currentPage + 1);
        }

        private void GoToPage(int targetPage)
        {
            if (_isAnimating)
                return;

            targetPage = Mathf.Clamp(targetPage, 0, _pageCount - 1);
            if (targetPage == _currentPage)
                return;

            if (_transitionRoutine != null)
                StopCoroutine(_transitionRoutine);

            _transitionRoutine = StartCoroutine(TransitionToPageRoutine(targetPage));
        }

        private void ApplyPageImmediate(int page)
        {
            page = Mathf.Clamp(page, 0, Mathf.Max(0, _pageCount - 1));
            _currentPage = page;

            var visible = BuildVisibilityMap(page);
            for (var i = 0; i < _cards.Count; i++)
            {
                var isVisible = visible[i];
                _cards[i].gameObject.SetActive(isVisible);
                _cards[i].anchoredPosition = Vector2.zero;

                var group = _cardGroups[i];
                group.alpha = isVisible ? 1f : 0f;
                group.interactable = isVisible;
                group.blocksRaycasts = isVisible;
            }

            if (cardRow != null)
                LayoutRebuilder.ForceRebuildLayoutImmediate(cardRow);

            RefreshVisibleHoverStates();
        }

        private IEnumerator TransitionToPageRoutine(int targetPage)
        {
            _isAnimating = true;
            SetNavigationInteractable(false);

            var fromPage = _currentPage;
            var fromVisible = BuildVisibilityMap(fromPage);
            var toVisible = BuildVisibilityMap(targetPage);

            if (cardRow != null)
                LayoutRebuilder.ForceRebuildLayoutImmediate(cardRow);

            var outgoing = BuildTweenStates(fromVisible, targetPage > fromPage ? -1f : 1f, transitionOffset, alphaFrom: 1f, alphaTo: 0f);
            yield return TweenCards(outgoing, transitionDuration);

            for (var i = 0; i < _cards.Count; i++)
            {
                if (!toVisible[i])
                {
                    _cards[i].gameObject.SetActive(false);
                    var group = _cardGroups[i];
                    group.alpha = 0f;
                    group.interactable = false;
                    group.blocksRaycasts = false;
                }
                else
                {
                    _cards[i].gameObject.SetActive(true);
                    var group = _cardGroups[i];
                    group.alpha = 0f;
                    group.interactable = false;
                    group.blocksRaycasts = false;
                }
            }

            if (cardRow != null)
                LayoutRebuilder.ForceRebuildLayoutImmediate(cardRow);

            var incoming = BuildTweenStates(toVisible, targetPage > fromPage ? 1f : -1f, transitionOffset, alphaFrom: 0f, alphaTo: 1f);
            yield return TweenCards(incoming, transitionDuration);

            _currentPage = targetPage;
            ApplyPageImmediate(_currentPage);
            RebuildPagination();
            UpdateArrowState();
            SetNavigationInteractable(true);

            _isAnimating = false;
            _transitionRoutine = null;
        }

        private List<CardTweenState> BuildTweenStates(bool[] visibleMap, float directionSign, float offset, float alphaFrom, float alphaTo)
        {
            var states = new List<CardTweenState>();
            for (var i = 0; i < _cards.Count; i++)
            {
                if (!visibleMap[i])
                    continue;

                var card = _cards[i];
                var group = _cardGroups[i];

                if (!card.gameObject.activeSelf)
                    card.gameObject.SetActive(true);

                var target = card.anchoredPosition;
                var start = target + new Vector2(directionSign * offset, 0f);

                if (Mathf.Approximately(alphaFrom, 1f) && Mathf.Approximately(alphaTo, 0f))
                {
                    start = target;
                    target = target + new Vector2(directionSign * offset, 0f);
                }

                card.anchoredPosition = start;
                group.alpha = alphaFrom;
                group.interactable = false;
                group.blocksRaycasts = false;

                states.Add(new CardTweenState(card, group, start, target, alphaFrom, alphaTo));
            }

            return states;
        }

        private IEnumerator TweenCards(List<CardTweenState> states, float duration)
        {
            if (states == null || states.Count == 0)
                yield break;

            if (duration <= 0.001f)
            {
                ApplyTweenStates(states, 1f);
                yield break;
            }

            var elapsed = 0f;
            while (elapsed < duration)
            {
                elapsed += useUnscaledTime ? Time.unscaledDeltaTime : Time.deltaTime;
                var t = Mathf.Clamp01(elapsed / duration);
                var eased = Mathf.SmoothStep(0f, 1f, t);
                ApplyTweenStates(states, eased);
                yield return null;
            }

            ApplyTweenStates(states, 1f);
        }

        private static void ApplyTweenStates(List<CardTweenState> states, float t)
        {
            for (var i = 0; i < states.Count; i++)
            {
                var state = states[i];
                state.Card.anchoredPosition = Vector2.LerpUnclamped(state.FromPosition, state.ToPosition, t);
                state.Group.alpha = Mathf.LerpUnclamped(state.FromAlpha, state.ToAlpha, t);
            }
        }

        private bool[] BuildVisibilityMap(int page)
        {
            var map = new bool[_cards.Count];
            var pageSize = Mathf.Max(1, visibleCardCount);
            var start = page * pageSize;
            var end = Mathf.Min(start + pageSize, _cards.Count);

            for (var i = start; i < end; i++)
                map[i] = true;

            return map;
        }

        private void RebuildPagination()
        {
            if (paginationBar == null)
                return;

            ClearPaginationItems();

            var tokens = BuildPageTokens();
            for (var i = 0; i < tokens.Count; i++)
            {
                var token = tokens[i];
                if (token.IsEllipsis)
                {
                    CreateEllipsisItem();
                    continue;
                }

                CreatePageButton(token.PageIndex, token.Label, token.PageIndex == _currentPage);
            }

            LayoutRebuilder.ForceRebuildLayoutImmediate(paginationBar);
        }

        private void ClearPaginationItems()
        {
            _pageButtons.Clear();

            for (var i = paginationBar.childCount - 1; i >= 0; i--)
                Destroy(paginationBar.GetChild(i).gameObject);
        }

        private List<PageToken> BuildPageTokens()
        {
            var tokens = new List<PageToken>();
            if (_pageCount <= 0)
                return tokens;

            var pages = new List<int>();
            if (_pageCount <= 7)
            {
                for (var i = 1; i <= _pageCount; i++)
                    pages.Add(i);
            }
            else
            {
                AddPageUnique(pages, 1);
                AddPageUnique(pages, _pageCount);
                AddPageUnique(pages, _currentPage);
                AddPageUnique(pages, _currentPage + 1);
                AddPageUnique(pages, _currentPage + 2);

                if (_currentPage <= 2)
                {
                    for (var i = 1; i <= 5; i++)
                        AddPageUnique(pages, i);
                }

                if (_currentPage >= _pageCount - 3)
                {
                    for (var i = _pageCount - 4; i <= _pageCount; i++)
                        AddPageUnique(pages, i);
                }
            }

            pages.Sort();

            var previous = -1;
            for (var i = 0; i < pages.Count; i++)
            {
                var pageOneBased = pages[i];
                if (previous != -1 && pageOneBased - previous > 1)
                    tokens.Add(PageToken.Ellipsis());

                tokens.Add(PageToken.Page(pageOneBased - 1, pageOneBased.ToString()));
                previous = pageOneBased;
            }

            return tokens;
        }

        private void AddPageUnique(List<int> pages, int oneBasedPage)
        {
            if (oneBasedPage < 1 || oneBasedPage > _pageCount)
                return;

            if (!pages.Contains(oneBasedPage))
                pages.Add(oneBasedPage);
        }

        private void CreatePageButton(int pageIndex, string label, bool isActive)
        {
            var go = new GameObject("Page_" + label, typeof(RectTransform), typeof(Image), typeof(Button));
            var rect = go.GetComponent<RectTransform>();
            rect.SetParent(paginationBar, false);
            rect.sizeDelta = new Vector2(pageItemWidth, pageItemHeight);

            var image = go.GetComponent<Image>();
            image.sprite = _pillSprite;
            image.type = _pillSprite != null ? Image.Type.Sliced : Image.Type.Simple;
            image.color = isActive ? pageActiveBackgroundColor : new Color(0f, 0f, 0f, 0f);
            image.raycastTarget = true;

            var button = go.GetComponent<Button>();
            button.targetGraphic = image;
            button.transition = Selectable.Transition.ColorTint;
            var colors = button.colors;
            colors.normalColor = Color.white;
            colors.highlightedColor = new Color(1f, 1f, 1f, 0.9f);
            colors.pressedColor = new Color(1f, 1f, 1f, 0.8f);
            colors.disabledColor = new Color(1f, 1f, 1f, 0.65f);
            colors.colorMultiplier = 1f;
            colors.fadeDuration = 0.08f;
            button.colors = colors;
            button.interactable = !isActive && !_isAnimating;

            var labelGo = new GameObject("Label", typeof(RectTransform), typeof(Text));
            var labelRt = labelGo.GetComponent<RectTransform>();
            labelRt.SetParent(rect, false);
            labelRt.anchorMin = Vector2.zero;
            labelRt.anchorMax = Vector2.one;
            labelRt.offsetMin = Vector2.zero;
            labelRt.offsetMax = Vector2.zero;

            var text = labelGo.GetComponent<Text>();
            text.font = _uiFont;
            text.text = label;
            text.alignment = TextAnchor.MiddleCenter;
            text.color = isActive ? pageActiveTextColor : pageTextColor;
            text.fontSize = 30;
            text.fontStyle = FontStyle.Bold;
            text.raycastTarget = false;

            var targetPage = pageIndex;
            button.onClick.AddListener(delegate { GoToPage(targetPage); });
            _pageButtons.Add(button);
        }

        private void CreateEllipsisItem()
        {
            var go = new GameObject("Page_Ellipsis", typeof(RectTransform), typeof(Text));
            var rect = go.GetComponent<RectTransform>();
            rect.SetParent(paginationBar, false);
            rect.sizeDelta = new Vector2(44f, pageItemHeight);

            var text = go.GetComponent<Text>();
            text.font = _uiFont;
            text.text = "...";
            text.alignment = TextAnchor.MiddleCenter;
            text.color = pageTextColor;
            text.fontSize = 30;
            text.fontStyle = FontStyle.Bold;
            text.raycastTarget = false;
        }

        private void UpdateArrowState()
        {
            if (leftArrowButton != null)
                leftArrowButton.interactable = !_isAnimating && _currentPage > 0;

            if (rightArrowButton != null)
                rightArrowButton.interactable = !_isAnimating && _currentPage < _pageCount - 1;
        }

        private void SetNavigationInteractable(bool interactable)
        {
            if (leftArrowButton != null)
                leftArrowButton.interactable = interactable && _currentPage > 0;

            if (rightArrowButton != null)
                rightArrowButton.interactable = interactable && _currentPage < _pageCount - 1;

            for (var i = 0; i < _pageButtons.Count; i++)
            {
                var button = _pageButtons[i];
                if (button == null)
                    continue;

                var isCurrent = button.name == "Page_" + (_currentPage + 1).ToString();
                button.interactable = interactable && !isCurrent;
            }
        }

        private void RefreshVisibleHoverStates()
        {
            for (var i = 0; i < _cards.Count; i++)
            {
                if (!_cards[i].gameObject.activeSelf)
                    continue;

                var hover = _cards[i].GetComponent<ScenarioCardHoverLift>();
                if (hover != null)
                    hover.RefreshBaseState();
            }
        }

        private readonly struct CardTweenState
        {
            public readonly RectTransform Card;
            public readonly CanvasGroup Group;
            public readonly Vector2 FromPosition;
            public readonly Vector2 ToPosition;
            public readonly float FromAlpha;
            public readonly float ToAlpha;

            public CardTweenState(
                RectTransform card,
                CanvasGroup group,
                Vector2 fromPosition,
                Vector2 toPosition,
                float fromAlpha,
                float toAlpha)
            {
                Card = card;
                Group = group;
                FromPosition = fromPosition;
                ToPosition = toPosition;
                FromAlpha = fromAlpha;
                ToAlpha = toAlpha;
            }
        }

        private readonly struct PageToken
        {
            public readonly bool IsEllipsis;
            public readonly int PageIndex;
            public readonly string Label;

            private PageToken(bool isEllipsis, int pageIndex, string label)
            {
                IsEllipsis = isEllipsis;
                PageIndex = pageIndex;
                Label = label;
            }

            public static PageToken Page(int pageIndex, string label)
            {
                return new PageToken(false, pageIndex, label);
            }

            public static PageToken Ellipsis()
            {
                return new PageToken(true, -1, "...");
            }
        }
    }
}
