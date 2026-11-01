"""Trazado de segmentos entre celdas mediante Bresenham."""


def trazar_rayo(fila_desde: int, col_desde: int, fila_hasta: int, col_hasta: int):
    """Devuelve las celdas del segmento, incluyendo ambos extremos."""
    f0 = int(fila_desde)
    c0 = int(col_desde)
    f1 = int(fila_hasta)
    c1 = int(col_hasta)

    dc = abs(c1 - c0)
    df = abs(f1 - f0)
    sc = 1 if c0 < c1 else -1
    sf = 1 if f0 < f1 else -1
    celdas = []
    error = dc - df
    f, c = f0, c0
    while True:
        celdas.append((f, c))
        if f == f1 and c == c1:
            break
        e2 = 2 * error
        if e2 > -df:
            error -= df
            c += sc
        if e2 < dc:
            error += dc
            f += sf
    return celdas
