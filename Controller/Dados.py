import time

class Dado:
    def __init__(self):
        self.TELA_INICIAL = 0
        self.TELA_EXECUCAO = 1
        self.TELA_CONFIGURACAO = 2

        self._telas = self.TELA_INICIAL
        self.full_scream = False

    @property
    def telas(self):
        return self._telas
    
    def set_telas(self,tela):
        self.print_status_tela(tela)
        self._telas = tela

    def print_status_tela(self, tela):
        if tela == self.TELA_INICIAL:
            print(f"Está na tela: INICIAL")
        elif tela == self.TELA_EXECUCAO:
            print(f"Está na tela: EXECUCAO")
        elif tela == self.TELA_CONFIGURACAO:
            print(f"Está na tela: CONFIGURACAO")

            