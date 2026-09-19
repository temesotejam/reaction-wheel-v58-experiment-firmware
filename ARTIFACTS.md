# Generated / reference artifacts from the original handoff package

元の `V58_IMPLEMENTATION_WITH_OPERATION_GUIDE_JA_20260919.zip` には、ソース・証跡に加えて以下3ファイルが含まれていました。GitHub版では生成物／参照資料をソース履歴と分けるため、実体はコミットしていません。

| Original path | Size (bytes) | SHA-256 | Role |
|---|---:|---|---|
| `binary/firmware.bin` | 1,347,952 | `6ceaade67fc4debf96612b9cc32693cf398af8f38049eac68a34c8326c241281` | 2026-09-13に記録されたapplication firmware build |
| `references/2026-09-15_dynamic_beta_to_walking_report_ja.pdf` | 990,214 | `c045daac119a7d0b8be32254d71a028e6e351cb2b4282cbded88147788270fb4` | 研究整理PDF |
| `references/2026-09-15_current_status_timeseries_ja.pdf` | 995,797 | `534ec249253666ed58f6070196480292ee72c38422b5f147dc503df4e52c9a13` | 研究整理PDF |

元パッケージ全体のファイルhashは [`provenance/ORIGINAL_PACKAGE_MANIFEST.json`](provenance/ORIGINAL_PACKAGE_MANIFEST.json) を参照してください。

`firmware.bin` を実機へ書き込む前には、可能ならこのリポジトリのソースからPlatformIOで再ビルドし、対象機体・revision・書込み結果を新しい証跡として残してください。
