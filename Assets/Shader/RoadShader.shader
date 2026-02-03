Shader "Custom/RoadShader_Separate_Width"
{
    Properties
    {
        _RoadColor ("Road Color", Color) = (0.5, 0.5, 0.5, 1)
        _LineColor ("Line Color", Color) = (1, 1, 1, 1)
        
        // [변경] 중앙선 두께와 가장자리 두께를 분리했습니다.
        _CenterWidth ("Center Line Width", Range(0, 0.5)) = 0.02
        _EdgeWidth ("Edge Line Width", Range(0, 0.5)) = 0.05 
        
        _DashFrequency ("Dash Frequency", Float) = 4
        _DashRatio ("Dash Ratio", Range(0, 1)) = 0.5
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

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

            fixed4 _RoadColor;
            fixed4 _LineColor;
            
            // [변경] 변수 분리
            float _CenterWidth;
            float _EdgeWidth;
            
            float _DashFrequency;
            float _DashRatio;

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                float2 uv = i.uv;

                // --- 가장자리 선 (이제 _EdgeWidth를 사용) ---
                // uv.x가 _EdgeWidth보다 작으면 왼쪽 선
                float leftEdge = step(uv.x, _EdgeWidth);
                // uv.x가 (1 - _EdgeWidth)보다 크면 오른쪽 선
                float rightEdge = step(1.0 - _EdgeWidth, uv.x);
                
                float edgeMask = leftEdge + rightEdge;

                // --- 중앙 점선 (이제 _CenterWidth를 사용) ---
                float centerLinePos = abs(uv.x - 0.5);
                float centerLineMask = step(centerLinePos, _CenterWidth * 0.5);
                
                // 점선 패턴 (타일링 유지)
                float dashPattern = step(frac(uv.y * _DashFrequency + (_DashRatio * 0.5)), _DashRatio);
                float finalCenterLine = centerLineMask * dashPattern;

                // --- 최종 합성 ---
                float finalMask = saturate(edgeMask + finalCenterLine);

                return lerp(_RoadColor, _LineColor, finalMask);
            }
            ENDCG
        }
    }
}