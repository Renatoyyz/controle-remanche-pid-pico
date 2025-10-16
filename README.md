# Controle de dispositivo de remanche em lanternas automotivas

Este projeto utiliza um Raspberry Pi com PyQt5 para criar uma interface de controle proporcional para um dispositivo de remanche a quente em lanternas automotivas. Abaixo estão as instruções para instalação das dependências, uso e operação do código.

## Instalação de Dependências

Para executar este projeto, você precisará instalar as seguintes dependências:

- Python 3.8 ou superior
- PyQt5

## Configuração do Ambiente Virtual

Para garantir que todas as dependências sejam instaladas corretamente e evitar conflitos com outras bibliotecas, é recomendável criar um ambiente virtual. Siga os passos abaixo para configurar um ambiente virtual e instalar as dependências a partir de um arquivo `requirements.txt`.

### Passo 1: Criação do Ambiente Virtual

Primeiro, crie um ambiente virtual utilizando o `venv`:

```bash
python -m venv .env
```

### Passo 2: Ativação do Ambiente Virtual

Ative o ambiente virtual. O comando para ativar o ambiente virtual varia conforme o sistema operacional:

- **Windows**:
    ```bash
    .\.env\Scripts\activate
    ```

- **macOS e Linux**:
    ```bash
    source .env/bin/activate
    ```

### Passo 3: Instalação das Dependências

Com o ambiente virtual ativado, instale as dependências listadas no arquivo `requirements.txt`:

```bash
pip install -r requirements.txt
```

### Passo 4: Desativação do Ambiente Virtual

Após terminar de usar o ambiente virtual, você pode desativá-lo com o comando:

```bash
deactivate
```

### Exemplo de Arquivo `requirements.txt`

Aqui está um exemplo de como o arquivo `requirements.txt` pode ser estruturado:

```
opencv-python
torch
matplotlib
ultralytics
```

Certifique-se de que o arquivo `requirements.txt` esteja no mesmo diretório onde você executa o comando `pip install -r requirements.txt`.
