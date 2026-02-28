using UnityEngine;

namespace GUI.Runtime
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(AudioSource))]
    public class UIButtonSfxRouter : MonoBehaviour
    {
        [SerializeField] private AudioClip clickClip;

        [Range(0f, 1f)]
        [SerializeField] private float clickVolume = 0.75f;

        private AudioSource _audioSource;

        private void Awake()
        {
            _audioSource = GetComponent<AudioSource>();
            _audioSource.playOnAwake = false;
            _audioSource.loop = false;
            _audioSource.spatialBlend = 0f;
        }

        public void SetClickClip(AudioClip click)
        {
            clickClip = click;
        }

        public void PlayClick()
        {
            PlayOneShot(clickClip, clickVolume);
        }

        private void PlayOneShot(AudioClip clip, float volume)
        {
            if (_audioSource == null || clip == null)
                return;

            _audioSource.PlayOneShot(clip, volume);
        }
    }
}
