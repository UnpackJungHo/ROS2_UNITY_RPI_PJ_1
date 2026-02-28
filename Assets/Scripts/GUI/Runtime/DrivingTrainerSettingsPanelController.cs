using System;
using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace GUI.Runtime
{
    [DisallowMultipleComponent]
    public class DrivingTrainerSettingsPanelController : MonoBehaviour
    {
        [Header("Entry")]
        [SerializeField] private Button openButton;

        [Header("Panel")]
        [SerializeField] private GameObject overlayRoot;
        [SerializeField] private Button backdropButton;
        [SerializeField] private Button closeButton;
        [SerializeField] private Button cancelButton;
        [SerializeField] private Button applyButton;

        [Header("Inputs")]
        [SerializeField] private Toggle laneAssistToggle;
        [SerializeField] private Toggle collisionAlertToggle;
        [SerializeField] private Toggle uiSoundToggle;
        [SerializeField] private Toggle fullscreenToggle;

        [Header("Preset")]
        [SerializeField] private Button presetSafeButton;
        [SerializeField] private Button presetBalancedButton;
        [SerializeField] private Button presetChallengeButton;
        [SerializeField] private TMP_Text presetValueText;

        [Header("Feedback")]
        [SerializeField] private TMP_Text applyHintText;

        private bool _isOpen;
        private bool _isDirty;
        private string _activePreset = "Balanced";

        private void Awake()
        {
            if (openButton == null)
                openButton = GetComponentInChildren<Button>(true);

            CloseImmediate();
            UpdatePresetVisual();
            UpdateHint();
        }

        private void OnEnable()
        {
            BindEvents();
        }

        private void OnDisable()
        {
            UnbindEvents();
        }

        private void Update()
        {
            if (_isOpen && Input.GetKeyDown(KeyCode.Escape))
                ClosePanel();
        }

        private void BindEvents()
        {
            UnbindEvents();

            if (openButton != null)
                openButton.onClick.AddListener(TogglePanel);

            if (backdropButton != null)
                backdropButton.onClick.AddListener(ClosePanel);

            if (closeButton != null)
                closeButton.onClick.AddListener(ClosePanel);

            if (cancelButton != null)
                cancelButton.onClick.AddListener(ClosePanel);

            if (applyButton != null)
                applyButton.onClick.AddListener(ApplySettings);

            if (presetSafeButton != null)
                presetSafeButton.onClick.AddListener(() => SetPreset("Safe"));

            if (presetBalancedButton != null)
                presetBalancedButton.onClick.AddListener(() => SetPreset("Balanced"));

            if (presetChallengeButton != null)
                presetChallengeButton.onClick.AddListener(() => SetPreset("Challenge"));

            RegisterToggle(laneAssistToggle);
            RegisterToggle(collisionAlertToggle);
            RegisterToggle(uiSoundToggle);
            RegisterToggle(fullscreenToggle);
        }

        private void UnbindEvents()
        {
            if (openButton != null)
                openButton.onClick.RemoveListener(TogglePanel);

            if (backdropButton != null)
                backdropButton.onClick.RemoveListener(ClosePanel);

            if (closeButton != null)
                closeButton.onClick.RemoveListener(ClosePanel);

            if (cancelButton != null)
                cancelButton.onClick.RemoveListener(ClosePanel);

            if (applyButton != null)
                applyButton.onClick.RemoveListener(ApplySettings);

            if (presetSafeButton != null)
                presetSafeButton.onClick.RemoveAllListeners();

            if (presetBalancedButton != null)
                presetBalancedButton.onClick.RemoveAllListeners();

            if (presetChallengeButton != null)
                presetChallengeButton.onClick.RemoveAllListeners();

            UnregisterToggle(laneAssistToggle);
            UnregisterToggle(collisionAlertToggle);
            UnregisterToggle(uiSoundToggle);
            UnregisterToggle(fullscreenToggle);
        }

        private void RegisterToggle(Toggle toggle)
        {
            if (toggle != null)
                toggle.onValueChanged.AddListener(OnAnyToggleChanged);
        }

        private void UnregisterToggle(Toggle toggle)
        {
            if (toggle != null)
                toggle.onValueChanged.RemoveListener(OnAnyToggleChanged);
        }

        private void TogglePanel()
        {
            if (_isOpen)
                ClosePanel();
            else
                OpenPanel();
        }

        private void OpenPanel()
        {
            if (overlayRoot == null)
                return;

            overlayRoot.SetActive(true);
            _isOpen = true;

            if (closeButton != null)
                EventSystem.current?.SetSelectedGameObject(closeButton.gameObject);
        }

        private void ClosePanel()
        {
            if (overlayRoot != null)
                overlayRoot.SetActive(false);

            _isOpen = false;
        }

        private void CloseImmediate()
        {
            if (overlayRoot != null)
                overlayRoot.SetActive(false);

            _isOpen = false;
        }

        private void SetPreset(string presetName)
        {
            if (string.Equals(_activePreset, presetName, StringComparison.Ordinal))
                return;

            _activePreset = presetName;
            MarkDirty();
            UpdatePresetVisual();
        }

        private void OnAnyToggleChanged(bool _)
        {
            MarkDirty();
        }

        private void MarkDirty()
        {
            _isDirty = true;
            UpdateHint();
        }

        private void ApplySettings()
        {
            _isDirty = false;
            UpdatePresetVisual();
            UpdateHint();

            ClosePanel();
        }

        private void UpdatePresetVisual()
        {
            SetPresetButtonState(presetSafeButton, string.Equals(_activePreset, "Safe", StringComparison.Ordinal));
            SetPresetButtonState(presetBalancedButton, string.Equals(_activePreset, "Balanced", StringComparison.Ordinal));
            SetPresetButtonState(presetChallengeButton, string.Equals(_activePreset, "Challenge", StringComparison.Ordinal));

            if (presetValueText != null)
                presetValueText.text = $"Active profile: {_activePreset}";
        }

        private void SetPresetButtonState(Button button, bool isActive)
        {
            if (button == null)
                return;

            var colors = button.colors;
            colors.normalColor = isActive ? Hex("24358F") : Hex("EEF2FF");
            colors.highlightedColor = Hex("6571B2");
            colors.selectedColor = colors.normalColor;
            colors.pressedColor = Hex("97A2D8");
            colors.colorMultiplier = 1f;
            button.colors = colors;

            var label = button.GetComponentInChildren<TMP_Text>(true);
            if (label != null)
                label.color = isActive ? Color.white : Hex("2C3E8A");
        }

        private void UpdateHint()
        {
            if (applyHintText == null)
                return;

            applyHintText.text = _isDirty
                ? "Unsaved changes"
                : "All settings are up to date";

            applyHintText.color = _isDirty ? Hex("8A5A2E") : Hex("5A6C9A");
        }

        private static Color Hex(string rgb)
        {
            if (ColorUtility.TryParseHtmlString("#" + rgb, out var c))
                return c;

            return Color.white;
        }
    }
}
