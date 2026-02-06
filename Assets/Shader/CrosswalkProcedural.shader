Shader "Custom/CrosswalkProceduralNoise"
{
    Properties
    {
        _MainColor ("Stripe Color", Color) = (0.9, 0.9, 0.9, 1) // 약간 회색빛
        _StripeCount ("Stripe Count", Float) = 10.0
        _StripeWidth ("Stripe Width Ratio", Range(0,1)) = 0.5
        _Fade ("Edge Fade", Range(0.001, 0.1)) = 0.02
        
        // --- 노이즈 관련 속성 추가 ---
        _NoiseScale ("Noise Scale", Float) = 150.0      // 노이즈 입자 크기 (클수록 자잘함)
        _NoiseStrength ("Noise Strength", Range(0, 1)) = 0.3 // 노이즈 강도 (높을수록 많이 벗겨짐)
    }
    SubShader
    {
        Tags { "Queue"="Transparent" "IgnoreProjector"="True" "RenderType"="Transparent" }
        LOD 100

        Blend SrcAlpha OneMinusSrcAlpha
        ZWrite Off

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            fixed4 _MainColor;
            float _StripeCount;
            float _StripeWidth;
            float _Fade;
            float _NoiseScale;
            float _NoiseStrength;

            // 간단한 랜덤 함수 (노이즈 생성용)
            // UV 좌표를 입력받아 0~1 사이의 난수를 반환합니다.
            float random (float2 uv)
            {
                return frac(sin(dot(uv, float2(12.9898, 78.233))) * 43758.5453123);
            }

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                // 1. 기본 줄무늬 패턴 생성 (이전과 동일)
                float pos = i.uv.x * _StripeCount;
                float pattern = frac(pos);
                float stripe = smoothstep(_StripeWidth, _StripeWidth + _Fade, pattern);
                stripe = 1.0 - stripe;
                float edgeFade = smoothstep(0.0, 0.1, i.uv.x) * smoothstep(1.0, 0.9, i.uv.x);

                // 2. 노이즈 생성
                // UV 좌표에 스케일을 곱해 자글자글한 노이즈를 만듭니다.
                float noiseVal = random(i.uv * _NoiseScale);

                // 3. 노이즈 적용
                // 원래의 줄무늬(stripe)에서 노이즈만큼을 살짝 뺍니다.
                // _NoiseStrength가 클수록 구멍이 많이 뚫린(많이 지워진) 느낌이 납니다.
                // clamp를 사용하여 값이 0 밑으로 내려가지 않도록 합니다.
                float finalAlpha = stripe - (noiseVal * _NoiseStrength);
                finalAlpha = clamp(finalAlpha, 0, 1);

                // 가장자리 페이드아웃 적용
                finalAlpha *= edgeFade;

                // 4. 최종 색상 반환
                fixed4 col = _MainColor;
                col.a *= finalAlpha; // 알파 채널에 노이즈가 반영된 값을 적용

                return col;
            }
            ENDCG
        }
    }
}