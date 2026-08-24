# RSSF aplicada à Agricultura de Precisão

Projeto acadêmico sobre o uso de uma Rede de Sensores Sem Fio (RSSF/WSN) para monitoramento de umidade do solo e condições ambientais, com apoio à irrigação inteligente.

## Estrutura

- `relatorio/Relatorio_RSSF_Agricultura_de_Precisao.pdf` — relatório final (5–8 páginas).
- `diagramas/arquitetura_rssf_agricultura.png` — diagrama da arquitetura proposta.
- `referencias/REFERENCIAS.md` — artigos científicos e links para consulta.
- `referencias/REFERENCIAS.bib` — referências em BibTeX.

## Arquitetura proposta

Nós sensores -> Gateway -> Servidor/Nuvem -> Painel do usuário.

Os nós podem medir:
- umidade do solo;
- temperatura do solo;
- temperatura do ar;
- umidade relativa;
- luminosidade;
- opcionalmente pH, chuva e nutrientes.

## Observação

O relatório apresenta uma proposta conceitual/experimental. Os valores de limiar de irrigação devem ser definidos de acordo com a cultura, tipo de solo, profundidade das raízes e condições locais.

## Como publicar no GitHub

1. Crie um novo repositório no GitHub, por exemplo `rssf-agricultura-precisao`.
2. Escolha a visibilidade solicitada pela disciplina.
3. Faça upload dos arquivos desta pasta.
4. Faça um commit com uma mensagem como `Projeto final - RSSF Agricultura de Precisão`.
5. Copie o endereço do repositório e envie pelo portal da disciplina.
